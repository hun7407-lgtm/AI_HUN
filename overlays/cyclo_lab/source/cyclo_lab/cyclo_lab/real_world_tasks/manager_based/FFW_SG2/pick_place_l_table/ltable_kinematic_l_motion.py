# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Kinematic L-motion for L-table mimic datagen (matches VR teleop root teleport)."""

from __future__ import annotations

import math

import torch

import isaaclab.utils.math as math_utils


class LTableKinematicLMotion:
    """Rotate the robot base ~90 deg, then drive forward while carrying the box."""

    def __init__(self, env, *, box_asset_name: str = "cardboard_box") -> None:
        self.env = env
        self._box_asset_name = box_asset_name
        self.reset()

    def reset(self) -> None:
        self._done = False
        self._rotation_active = False
        self._forward_active = False
        self._carry_box = False
        self._box_rel_pos = None
        self._box_rel_quat = None
        self._carry_env_mask = None
        self._rotation_start_yaw = 0.0
        self._rotation_target_yaw = 0.0
        self._rotation_sim_alpha = 0.0
        self._forward_start_pos = None
        self._forward_end_pos = None
        self._forward_yaw = 0.0
        self._forward_sim_alpha = 0.0
        self._read_cfg()

    def _read_cfg(self) -> None:
        cfg = self.env.cfg
        self._face_left_yaw = float(getattr(cfg, "teleop_l_yaw", math.pi / 2.0))
        self._forward_distance_m = float(getattr(cfg, "teleop_l_forward_m", 0.30))
        self._rotation_duration_s = float(getattr(cfg, "teleop_l_rotation_duration_s", 3.0))
        self._forward_duration_s = float(getattr(cfg, "teleop_l_forward_duration_s", 2.0))
        self._sim_dt = float(self.env.physics_dt * self.env.cfg.decimation)

    def is_active(self) -> bool:
        return self._rotation_active or self._forward_active

    def is_done(self) -> bool:
        return self._done

    def step_interval(self, env_ids: torch.Tensor) -> None:
        """Called once per env control step from an interval event."""
        if self._done:
            return
        if self.is_active():
            if self._rotation_active:
                self._step_yaw_rotation(env_ids)
            elif self._forward_active:
                self._step_forward_motion(env_ids)
            return
        grasped = self._dual_grasped_mask(env_ids)
        if grasped.any():
            self._capture_box_carry(env_ids[grasped])
            self._begin_yaw_rotation(env_ids)

    def _dual_grasped_mask(self, env_ids: torch.Tensor) -> torch.Tensor:
        """True per env when both grippers hold the box (closed + near eef midpoint)."""
        return self._grasp_latch_candidates(env_ids)

    def _grasp_latch_candidates(
        self, env_ids: torch.Tensor, *, diff_threshold: float = 0.18
    ) -> torch.Tensor:
        """Both grippers closed and box near each eef (strict bimanual grasp)."""
        closed = self._grippers_closed_mask(env_ids)
        if not closed.any():
            return torch.zeros(len(env_ids), dtype=torch.bool, device=self.env.device)
        left_eef = self.env.scene["left_eef"]
        right_eef = self.env.scene["right_eef"]
        box = self.env.scene[self._box_asset_name]
        box_pos = box.data.root_pos_w[env_ids]
        left_pos = left_eef.data.target_pos_w[env_ids, 0, :]
        right_pos = right_eef.data.target_pos_w[env_ids, 0, :]
        midpoint = (left_pos + right_pos) * 0.5
        near_left = torch.linalg.vector_norm(box_pos - left_pos, dim=1) < diff_threshold
        near_right = torch.linalg.vector_norm(box_pos - right_pos, dim=1) < diff_threshold
        near_midpoint = torch.linalg.vector_norm(box_pos - midpoint, dim=1) < diff_threshold
        return closed & near_left & near_right & near_midpoint

    def clear_carry(self, env_ids: torch.Tensor) -> None:
        self._ensure_carry_buffers()
        self._carry_env_mask[env_ids] = False
        self._carry_box = False

    def _dual_grasped(self) -> bool:
        env_ids = torch.arange(self.env.num_envs, device=self.env.device)
        return bool(self._dual_grasped_mask(env_ids).any())

    def _grippers_closed_mask(self, env_ids: torch.Tensor, *, threshold: float = 0.2) -> torch.Tensor:
        robot = self.env.scene["robot"]
        left_i = robot.joint_names.index("gripper_l_joint1")
        right_i = robot.joint_names.index("gripper_r_joint1")
        left = robot.data.joint_pos[env_ids, left_i]
        right = robot.data.joint_pos[env_ids, right_i]
        return (left >= threshold) & (right >= threshold)

    def update_carry_latch(self, env_ids: torch.Tensor) -> None:
        """Latch carry only on a confirmed bimanual grasp (before base L-motion moves)."""
        self._ensure_carry_buffers()
        open_grippers = ~self._grippers_closed_mask(env_ids)
        if open_grippers.any():
            self.clear_carry(env_ids[open_grippers])
        grasped = self._grasp_latch_candidates(env_ids)
        if not grasped.any():
            return
        newly_grasped = env_ids[grasped & ~self._carry_env_mask[env_ids]]
        if len(newly_grasped) > 0:
            self._capture_box_carry(newly_grasped)

    def apply_latched_carry(self, env_ids: torch.Tensor) -> None:
        """Apply latched carry through rotation/drive; release when grippers open."""
        self._ensure_carry_buffers()
        grippers_closed = self._grippers_closed_mask(env_ids)
        releasing = env_ids[(~grippers_closed) & self._carry_env_mask[env_ids]]
        if len(releasing) > 0:
            self._carry_env_mask[releasing] = False
        carrying = env_ids[self._carry_env_mask[env_ids] & grippers_closed]
        if len(carrying) > 0:
            self._apply_box_carry(carrying)

    def is_l_motion_latched(self, env_ids: torch.Tensor) -> torch.Tensor:
        """True when a bimanual grasp has been latched and grippers remain closed."""
        self._ensure_carry_buffers()
        return self._carry_env_mask[env_ids] & self._grippers_closed_mask(env_ids)

    def sync_carry_with_grasp(self, env_ids: torch.Tensor) -> None:
        """After robot base moves: keep latched carry while grippers stay closed."""
        self.apply_latched_carry(env_ids)

    def _ensure_carry_buffers(self) -> None:
        if self._box_rel_pos is not None:
            return
        n = self.env.num_envs
        device = self.env.device
        self._box_rel_pos = torch.zeros(n, 3, device=device)
        self._box_rel_quat = torch.zeros(n, 4, device=device)
        self._box_rel_quat[:, 0] = 1.0
        self._carry_env_mask = torch.zeros(n, dtype=torch.bool, device=device)

    def _capture_box_carry(self, env_ids: torch.Tensor) -> None:
        self._ensure_carry_buffers()
        robot = self.env.scene["robot"]
        box = self.env.scene[self._box_asset_name]
        root_pos = robot.data.root_pos_w[env_ids, :3]
        root_quat = robot.data.root_quat_w[env_ids]
        box_pos = box.data.root_pos_w[env_ids, :3]
        box_quat = box.data.root_quat_w[env_ids]
        inv_root_quat = math_utils.quat_inv(root_quat)
        self._box_rel_pos[env_ids] = math_utils.quat_apply(inv_root_quat, box_pos - root_pos)
        self._box_rel_quat[env_ids] = math_utils.quat_mul(inv_root_quat, box_quat)
        self._carry_env_mask[env_ids] = True
        self._carry_box = True

    def _apply_box_carry(self, env_ids: torch.Tensor) -> None:
        self._ensure_carry_buffers()
        active_ids = env_ids[self._carry_env_mask[env_ids]]
        if len(active_ids) == 0:
            return
        robot = self.env.scene["robot"]
        box = self.env.scene[self._box_asset_name]
        root_pos = robot.data.root_pos_w[active_ids, :3]
        root_quat = robot.data.root_quat_w[active_ids]
        rel_pos = self._box_rel_pos[active_ids]
        rel_quat = self._box_rel_quat[active_ids]
        box_pos = root_pos + math_utils.quat_apply(root_quat, rel_pos)
        box_quat = math_utils.quat_mul(root_quat, rel_quat)
        box_pose = torch.cat([box_pos, box_quat], dim=-1)
        box.write_root_pose_to_sim(box_pose, env_ids=active_ids)
        box.write_root_velocity_to_sim(torch.zeros(len(active_ids), 6, device=self.env.device), env_ids=active_ids)

    def _get_robot_yaw(self, env_ids: torch.Tensor) -> float:
        robot = self.env.scene["robot"]
        _, _, yaw = math_utils.euler_xyz_from_quat(robot.data.root_quat_w[env_ids])
        return float(yaw[0].item())

    def _begin_yaw_rotation(self, env_ids: torch.Tensor) -> None:
        self._rotation_start_yaw = self._get_robot_yaw(env_ids)
        self._rotation_target_yaw = self._face_left_yaw
        self._rotation_sim_alpha = 0.0
        self._rotation_active = True

    def _begin_forward_motion(self, env_ids: torch.Tensor) -> None:
        robot = self.env.scene["robot"]
        device = self.env.device
        self._forward_start_pos = robot.data.root_pos_w[env_ids, :3].clone()
        quat = robot.data.root_quat_w[env_ids].clone()
        offset = torch.tensor([[self._forward_distance_m, 0.0, 0.0]], device=device).expand(len(env_ids), -1)
        self._forward_end_pos = self._forward_start_pos + math_utils.quat_apply(quat, offset)
        self._forward_yaw = self._get_robot_yaw(env_ids)
        self._forward_sim_alpha = 0.0
        self._forward_active = True

    @staticmethod
    def _smoothstep(alpha: float) -> float:
        return alpha * alpha * (3.0 - 2.0 * alpha)

    @staticmethod
    def _lerp_yaw(start: float, end: float, alpha: float) -> float:
        delta = (end - start + math.pi) % (2.0 * math.pi) - math.pi
        return start + alpha * delta

    def _set_robot_pose(self, env_ids: torch.Tensor, pos: torch.Tensor, yaw: float) -> None:
        robot = self.env.scene["robot"]
        device = self.env.device
        quat = math_utils.quat_from_euler_xyz(
            torch.zeros(len(env_ids), device=device),
            torch.zeros(len(env_ids), device=device),
            torch.full((len(env_ids),), yaw, device=device),
        )
        root_pose = torch.cat([pos, quat], dim=-1)
        robot.write_root_pose_to_sim(root_pose, env_ids=env_ids)
        robot.write_root_velocity_to_sim(torch.zeros(len(env_ids), 6, device=device), env_ids=env_ids)

    def _step_yaw_rotation(self, env_ids: torch.Tensor) -> None:
        self._rotation_sim_alpha = min(1.0, self._rotation_sim_alpha + self._sim_dt / self._rotation_duration_s)
        alpha = self._smoothstep(self._rotation_sim_alpha)
        yaw = self._lerp_yaw(self._rotation_start_yaw, self._rotation_target_yaw, alpha)
        robot = self.env.scene["robot"]
        self._set_robot_pose(env_ids, robot.data.root_pos_w[env_ids, :3], yaw)
        self.sync_carry_with_grasp(env_ids)
        if alpha >= 1.0:
            self._rotation_active = False
            self._begin_forward_motion(env_ids)

    def _step_forward_motion(self, env_ids: torch.Tensor) -> None:
        self._forward_sim_alpha = min(1.0, self._forward_sim_alpha + self._sim_dt / self._forward_duration_s)
        alpha = self._smoothstep(self._forward_sim_alpha)
        pos = self._forward_start_pos + alpha * (self._forward_end_pos - self._forward_start_pos)
        self._set_robot_pose(env_ids, pos, self._forward_yaw)
        self.sync_carry_with_grasp(env_ids)
        if alpha >= 1.0:
            self._forward_active = False
            self._carry_box = False
            self._done = True
