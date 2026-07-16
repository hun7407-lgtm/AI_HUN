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

"""Swerve-drive inverse kinematics for the FFW three-module base.

Turns a chassis twist ``(vx, vy, omega)`` into per-module steer angles and wheel speeds.
Only useful on a robot whose base is actually free to move -- see
:mod:`cyclo_lab.assets.robots.FFW_SG2_MOBILE`. The stock ``FFW_SG2`` is welded to the
world and moves by kinematic root teleport instead.
"""

from __future__ import annotations

import math
import torch

from cyclo_lab.assets.robots.FFW_SG2 import (
    SG2_SWERVE_MODULE_ANGLE_OFFSETS,
    SG2_SWERVE_MODULE_X_OFFSETS,
    SG2_SWERVE_MODULE_Y_OFFSETS,
    SG2_SWERVE_STEERING_JOINTS,
    SG2_SWERVE_WHEEL_JOINTS,
    SG2_SWERVE_WHEEL_RADIUS,
)


def _module_key(joint_name: str) -> str:
    """``left_wheel_steer`` -> ``left``."""
    return joint_name.split("_")[0]


def _wrap_to_pi(angle: torch.Tensor) -> torch.Tensor:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class SwerveController:
    """Chassis twist -> steer/drive joint targets for a three-module swerve base.

    Args:
        robot: the articulation to drive.
        wheel_radius: drive wheel radius in metres.
        optimize_steer: take the shorter steer path by flipping the wheel direction when
            the target is more than 90 deg away. Without this, reversing makes each module
            swing a full 180 deg.

    Example:
        swerve = SwerveController(robot)
        swerve.apply(vx=0.5, vy=0.0, omega=0.0)   # robot frame, m/s and rad/s
        robot.write_data_to_sim()
    """

    def __init__(
        self,
        robot,
        wheel_radius: float = SG2_SWERVE_WHEEL_RADIUS,
        optimize_steer: bool = True,
    ):
        self._robot = robot
        self._wheel_radius = wheel_radius
        self._optimize_steer = optimize_steer

        # find_joints does NOT preserve the order it is given -- it returns articulation
        # order. Map the module geometry by name or the wheels get another module's offset.
        self._steer_ids, steer_names = robot.find_joints(list(SG2_SWERVE_STEERING_JOINTS))
        self._drive_ids, drive_names = robot.find_joints(list(SG2_SWERVE_WHEEL_JOINTS))

        geometry = {
            _module_key(name): (x, y, a)
            for name, x, y, a in zip(
                SG2_SWERVE_STEERING_JOINTS,
                SG2_SWERVE_MODULE_X_OFFSETS,
                SG2_SWERVE_MODULE_Y_OFFSETS,
                SG2_SWERVE_MODULE_ANGLE_OFFSETS,
            )
        }
        steer_keys = [_module_key(n) for n in steer_names]
        drive_keys = [_module_key(n) for n in drive_names]
        if steer_keys != drive_keys:
            raise ValueError(f"steer/drive module order differs: {steer_keys} vs {drive_keys}")

        device = robot.device
        self._module_x = torch.tensor([geometry[k][0] for k in steer_keys], device=device)
        self._module_y = torch.tensor([geometry[k][1] for k in steer_keys], device=device)
        self._angle_offset = torch.tensor([geometry[k][2] for k in steer_keys], device=device)
        self._module_keys = steer_keys
        self.steer_names = steer_names
        self.drive_names = drive_names

    @property
    def module_keys(self) -> list[str]:
        return list(self._module_keys)

    def compute(self, vx: float, vy: float, omega: float) -> tuple[torch.Tensor, torch.Tensor]:
        """Chassis twist (robot frame) -> (steer angles [rad], wheel speeds [rad/s])."""
        # Velocity each module must trace: chassis velocity + omega x r.
        mvx = vx - omega * self._module_y
        mvy = vy + omega * self._module_x

        angles = torch.atan2(mvy, mvx) - self._angle_offset
        speeds = torch.hypot(mvx, mvy) / self._wheel_radius

        if self._optimize_steer:
            current = self._robot.data.joint_pos[0, self._steer_ids]
            delta = _wrap_to_pi(angles - current)
            flip = delta.abs() > (math.pi / 2.0)
            # Steering the other way round and reversing the wheel lands the same place.
            angles = torch.where(flip, _wrap_to_pi(angles + math.pi), angles)
            speeds = torch.where(flip, -speeds, speeds)
            # Track the continuous joint rather than snapping across the +/-pi seam.
            angles = current + _wrap_to_pi(angles - current)

        return angles.unsqueeze(0), speeds.unsqueeze(0)

    def apply(self, vx: float, vy: float, omega: float) -> tuple[torch.Tensor, torch.Tensor]:
        """Compute and write the joint targets. Caller still calls ``write_data_to_sim``."""
        angles, speeds = self.compute(vx, vy, omega)
        self._robot.set_joint_position_target(angles, joint_ids=self._steer_ids)
        self._robot.set_joint_velocity_target(speeds, joint_ids=self._drive_ids)
        return angles, speeds

    def apply_world(self, vx: float, vy: float, omega: float):
        """Same as :meth:`apply` but the twist is in world frame.

        ``apply`` speaks the robot's frame: "forward" follows the robot as it turns. Use
        this when the command means a fixed world direction regardless of heading.
        """
        from isaaclab.utils.math import euler_xyz_from_quat

        _, _, yaw = euler_xyz_from_quat(self._robot.data.root_quat_w)
        yaw = yaw[0].item()
        cos_y, sin_y = math.cos(-yaw), math.sin(-yaw)
        return self.apply(vx * cos_y - vy * sin_y, vx * sin_y + vy * cos_y, omega)

    def stop(self):
        zeros_drive = torch.zeros((1, len(self._drive_ids)), device=self._robot.device)
        current = self._robot.data.joint_pos[0, self._steer_ids].unsqueeze(0)
        self._robot.set_joint_velocity_target(zeros_drive, joint_ids=self._drive_ids)
        self._robot.set_joint_position_target(current, joint_ids=self._steer_ids)
