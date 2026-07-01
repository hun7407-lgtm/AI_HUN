# Copyright (c) 2024-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

import torch
from collections.abc import Sequence

import isaaclab.utils.math as PoseUtils
from isaaclab.envs import ManagerBasedRLMimicEnv, ManagerBasedRLEnvCfg


class FFWSG2PickPlaceMimicEnv(ManagerBasedRLMimicEnv):
    """
    Isaac Lab Mimic environment wrapper class for FFW SG2 Pick and Place.
    """

    # ActionManager term order: [arms..., gripper_r, lift, head]
    # joint_pos / joint_pos_target obs order: [arms..., gripper_r, head1, lift, head2]
    _OBS_HEAD1_SLICE = slice(16, 17)
    _OBS_LIFT_SLICE = slice(17, 18)
    _OBS_HEAD2_SLICE = slice(18, 19)

    def __init__(self, cfg: ManagerBasedRLEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        self.robot_root_pos = self.scene['robot'].data.root_pos_w
        self.robot_root_quat = self.scene['robot'].data.root_quat_w

    @classmethod
    def _lift_head_from_joint_obs(cls, joint_row: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """Map obs joint layout (head1, lift, head2) to action layout (lift, head)."""
        lift = joint_row[cls._OBS_LIFT_SLICE]
        head = torch.cat([joint_row[cls._OBS_HEAD1_SLICE], joint_row[cls._OBS_HEAD2_SLICE]], dim=0)
        return lift, head

    def _resolve_lift_head_actions(self, joint_row: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """Use replayed lift/head from mimic datagen when available."""
        cmd = getattr(self, "_mimic_body_joint_cmd", None)
        if cmd is not None:
            cmd = cmd.to(joint_row.device, dtype=joint_row.dtype).reshape(-1)
            return cmd[0:1], cmd[1:3]
        return self._lift_head_from_joint_obs(joint_row)

    def _inject_mimic_body_joints(self, action: torch.Tensor) -> torch.Tensor:
        """Force VR lift/head targets into the flat action vector (lift@16, head@17:19)."""
        cmd = getattr(self, "_mimic_body_joint_cmd", None)
        if cmd is None:
            return action
        out = action.clone()
        cmd = cmd.reshape(-1).to(device=out.device, dtype=out.dtype)
        if out.dim() == 1:
            out[16] = cmd[0]
            out[17] = cmd[1]
            out[18] = cmd[2]
        else:
            out[:, 16] = cmd[0]
            out[:, 17] = cmd[1]
            out[:, 18] = cmd[2]
        return out

    def step(self, action: torch.Tensor):
        return super().step(self._inject_mimic_body_joints(action))

    def get_robot_eef_pose(self, eef_name: str, env_ids: Sequence[int] | None = None) -> torch.Tensor:
        if env_ids is None:
            env_ids = slice(None)

        # Support both left and right arm based on eef_name
        # eef_name can be: "left_arm", "right_arm", or robot name (defaults to right arm)
        if "right" in eef_name.lower():
            # Default to right arm (primary manipulator for Mimic tracking)
            eef_pose = self.obs_buf["policy"]["right_eef_pose"][env_ids]
        elif "left" in eef_name.lower():
            eef_pose = self.obs_buf["policy"]["left_eef_pose"][env_ids]
        else:
            print("Defaulting to right arm EEF state.")
            eef_pose = self.obs_buf["policy"]["right_eef_pose"][env_ids]

        eef_pos = eef_pose[:, :3]
        eef_quat = eef_pose[:, 3:7]
        # quat: (w, x, y, z)
        eef_pose = PoseUtils.make_pose(eef_pos, PoseUtils.matrix_from_quat(eef_quat))

        return eef_pose

    def target_eef_pose_to_action(
        self,
        target_eef_pose_dict: dict,
        gripper_action_dict: dict,
        action_noise_dict: dict | None = None,
        env_id: int = 0,
    ) -> torch.Tensor:
        if len(self.cfg.subtask_configs) > 1:
            return self._dual_arm_target_to_action(target_eef_pose_dict, gripper_action_dict, env_id)

        # Single-EEF legacy path
        eef_name = list(self.cfg.subtask_configs.keys())[0]

        if "right" in eef_name.lower():
            target_right_eef_pose = target_eef_pose_dict[eef_name]
            right_gripper_action = gripper_action_dict[eef_name]

            # Convert right EEF pose to pos + quat
            right_eef_pos, right_eef_rot = PoseUtils.unmake_pose(target_right_eef_pose)
            right_eef_quat = PoseUtils.quat_from_matrix(right_eef_rot)

            right_pose_action = torch.cat([right_eef_pos, right_eef_quat], dim=0)

            # For left arm, keep current observation state (not controlled by Mimic trajectory)
            # Get current left arm EEF state from observation
            left_eef_pose = self.obs_buf["policy"]["left_eef_pose"]

            if left_eef_pose.dim() > 1:
                left_eef_pose = left_eef_pose[env_id]
            left_pose_action = left_eef_pose[:7]  # pos(3) + quat(4)

            # For gripper_l, keep current state
            joint_pos_target = self.obs_buf["policy"]["joint_pos_target"]

            if joint_pos_target.dim() > 1:
                left_gripper_action = joint_pos_target[env_id, 7:8]
                lift_action, head_action = self._resolve_lift_head_actions(joint_pos_target[env_id])
            else:
                left_gripper_action = joint_pos_target[7:8]
                lift_action, head_action = self._resolve_lift_head_actions(joint_pos_target)

            # Concatenate full 19D IK action:
            # [left_eef(7), gripper_l(1), right_eef(7), gripper_r(1), lift(1), head(2)]
            action = torch.cat([
                left_pose_action,     # 1-7: left arm (keep current)
                left_gripper_action,  # 8: left gripper (keep current)
                right_pose_action,      # 9-15: right arm (Mimic controlled)
                right_gripper_action,      # 16: right gripper (keep current)
                lift_action,           # 17: lift (keep current)
                head_action,           # 18-19: head (keep current)
            ], dim=0)

            result = action.unsqueeze(0)
        elif "left" in eef_name.lower():
            target_left_eef_pose = target_eef_pose_dict[eef_name]
            left_gripper_action = gripper_action_dict[eef_name]

            # Convert left EEF pose to pos + quat
            left_eef_pos, left_eef_rot = PoseUtils.unmake_pose(target_left_eef_pose)
            left_eef_quat = PoseUtils.quat_from_matrix(left_eef_rot)

            left_pose_action = torch.cat([left_eef_pos, left_eef_quat], dim=0)

            # For right arm, keep current observation state (not controlled by Mimic trajectory)
            # Get current right arm EEF state from observation
            right_eef_pose = self.obs_buf["policy"]["right_eef_pose"]

            if right_eef_pose.dim() > 1:
                right_eef_pose = right_eef_pose[env_id]
            right_pose_action = right_eef_pose[:7]  # pos(3) + quat(4)

            # For gripper_l, keep current state
            joint_pos_target = self.obs_buf["policy"]["joint_pos_target"]

            if joint_pos_target.dim() > 1:
                right_gripper_action = joint_pos_target[env_id, 15:16]
                lift_action, head_action = self._resolve_lift_head_actions(joint_pos_target[env_id])
            else:
                right_gripper_action = joint_pos_target[15:16]
                lift_action, head_action = self._resolve_lift_head_actions(joint_pos_target)

            # Concatenate full 19D IK action:
            # [left_eef(7), gripper_l(1), right_eef(7), gripper_r(1), lift(1), head(2)]
            action = torch.cat([
                left_pose_action,     # 1-7: left arm (Mimic controlled)
                left_gripper_action,  # 8: left gripper (Mimic controlled)
                right_pose_action,      # 9-15: right arm (keep current)
                right_gripper_action,      # 16: right gripper (keep current)
                lift_action,           # 17: lift (keep current)
                head_action,           # 18-19: head (keep current)
            ], dim=0)

            result = action.unsqueeze(0)
        return result

    def _dual_arm_target_to_action(
        self,
        target_eef_pose_dict: dict,
        gripper_action_dict: dict,
        env_id: int,
    ) -> torch.Tensor:
        """Build 19D IK action controlling both arms from mimic waypoint dicts."""
        joint_pos_target = self.obs_buf["policy"]["joint_pos_target"]
        if joint_pos_target.dim() > 1:
            joint_row = joint_pos_target[env_id]
        else:
            joint_row = joint_pos_target

        def _eef_action(eef_name: str, fallback_obs_key: str) -> torch.Tensor:
            if eef_name in target_eef_pose_dict:
                pos, rot = PoseUtils.unmake_pose(target_eef_pose_dict[eef_name])
                return torch.cat([pos, PoseUtils.quat_from_matrix(rot)], dim=0)
            obs_pose = self.obs_buf["policy"][fallback_obs_key]
            return obs_pose[env_id, :7] if obs_pose.dim() > 1 else obs_pose[:7]

        left_pose = _eef_action("left_arm", "left_eef_pose")
        right_pose = _eef_action("right_arm", "right_eef_pose")
        left_gripper = gripper_action_dict.get("left_arm", joint_row[7:8])
        right_gripper = gripper_action_dict.get("right_arm", joint_row[15:16])
        lift_action, head_action = self._resolve_lift_head_actions(joint_row)

        action = torch.cat(
            [left_pose, left_gripper, right_pose, right_gripper, lift_action, head_action],
            dim=0,
        )
        return action.unsqueeze(0)

    def action_to_target_eef_pose(self, action: torch.Tensor) -> dict[str, torch.Tensor]:
        if len(self.cfg.subtask_configs) > 1:
            left_pos = action[:, 0:3]
            left_quat = action[:, 3:7]
            right_pos = action[:, 8:11]
            right_quat = action[:, 11:15]
            return {
                "left_arm": PoseUtils.make_pose(left_pos, PoseUtils.matrix_from_quat(left_quat)).clone(),
                "right_arm": PoseUtils.make_pose(right_pos, PoseUtils.matrix_from_quat(right_quat)).clone(),
            }

        eef_name = list(self.cfg.subtask_configs.keys())[0]

        # For FFW-SG2, use right arm as primary manipulator
        # Action format from IK conversion: [left_eef(7), gripper_l(1), right_eef(7), gripper_r(1), head(2), lift(1)]
        # We return only the right arm EEF pose (indices 8-14)
        if "right" in eef_name.lower():
            target_eef_pos = action[:, 8:11]    # Right arm position
            target_eef_quat = action[:, 11:15]  # Right arm quaternion
            target_eef_rot = PoseUtils.matrix_from_quat(target_eef_quat)
        elif "left" in eef_name.lower():
            target_eef_pos = action[:, 0:3]    # Left arm position
            target_eef_quat = action[:, 3:7]  # Left arm quaternion
            target_eef_rot = PoseUtils.matrix_from_quat(target_eef_quat)
        else:
            print("Defaulting to right arm EEF state.")
            target_eef_pos = action[:, 8:11]    # Right arm position
            target_eef_quat = action[:, 11:15]  # Right arm quaternion
            target_eef_rot = PoseUtils.matrix_from_quat(target_eef_quat)

        target_eef_pose = PoseUtils.make_pose(target_eef_pos, target_eef_rot).clone()

        return {eef_name: target_eef_pose}

    def actions_to_gripper_actions(self, actions: torch.Tensor) -> dict[str, torch.Tensor]:
        if len(self.cfg.subtask_configs) > 1:
            return {"left_arm": actions[:, 7:8], "right_arm": actions[:, 15:16]}

        eef_name = list(self.cfg.subtask_configs.keys())[0]

        # For FFW-SG2, return right gripper (index 15)
        # Action format: [left_eef(7), gripper_l(1), right_eef(7), gripper_r(1), head(2), lift(1)]
        if "right" in eef_name.lower():
            target_gripper_action = actions[:, 15:16]
        elif "left" in eef_name.lower():
            target_gripper_action = actions[:, 7:8]
        else:
            print("Defaulting to right gripper action.")
            target_gripper_action = actions[:, 15:16]
        return {eef_name: target_gripper_action}

    def get_subtask_term_signals(self, env_ids: Sequence[int] | None = None) -> dict[str, torch.Tensor]:
        """
        Gets a dictionary of termination signal flags for each subtask in a task. The flag is 1
        when the subtask has been completed and 0 otherwise. The implementation of this method is
        required if intending to enable automatic subtask term signal annotation when running the
        dataset annotation tool. This method can be kept unimplemented if intending to use manual
        subtask term signal annotation.

        Args:
            env_ids: Environment indices to get the termination signals for. If None, all envs are considered.

        Returns:
            A dictionary termination signal flags (False or True) for each subtask.
        """
        if env_ids is None:
            env_ids = slice(None)

        signals = dict()
        subtask_terms = self.obs_buf["subtask_terms"]
        for term_name, term_signal in subtask_terms.items():
            signals[term_name] = term_signal[env_ids]

        return signals
