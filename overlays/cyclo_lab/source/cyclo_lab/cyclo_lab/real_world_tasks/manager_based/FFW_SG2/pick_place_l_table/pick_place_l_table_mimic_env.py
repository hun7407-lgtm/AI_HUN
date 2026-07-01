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

from __future__ import annotations

import time

import torch

from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.envs.common import VecEnvStepReturn
from isaaclab.utils.datasets import EpisodeData

from cyclo_lab.real_world_tasks.manager_based.FFW_SG2.pick_place.pick_place_mimic_env import (
    FFWSG2PickPlaceMimicEnv,
)

from .ltable_kinematic_l_motion import LTableKinematicLMotion


def _state_to_device(state: dict, device: torch.device) -> dict:
    output: dict = {}
    for asset_type, assets in state.items():
        output[asset_type] = {}
        for asset_name, fields in assets.items():
            output[asset_type][asset_name] = {
                key: value.to(device) if isinstance(value, torch.Tensor) else value
                for key, value in fields.items()
            }
    return output


def apply_recorded_step_state(env, episode: EpisodeData, state_index: int, env_ids: torch.Tensor) -> bool:
    """Apply recorded post-step scene state (same as demo playback)."""
    step_state = episode.get_state(state_index)
    if step_state is None:
        return False
    step_state = _state_to_device(step_state, env.device)
    env.scene.reset_to(step_state, env_ids=env_ids, is_relative=True)
    env.sim.forward()
    return True


def _expand_state_batch(tensor: torch.Tensor, batch_size: int) -> torch.Tensor:
    """Broadcast single-env recorded tensors across vectorized envs."""
    if tensor.shape[0] == batch_size:
        return tensor.clone()
    if tensor.shape[0] == 1:
        return tensor.expand(batch_size, -1).clone()
    raise ValueError(f"Cannot broadcast state tensor shape {tensor.shape} to {batch_size} envs")


def apply_recorded_robot_root_state(env, episode: EpisodeData, state_index: int, env_ids: torch.Tensor) -> bool:
    """Replay recorded robot base pose for L-motion (box handled via rigid carry)."""
    step_state = episode.get_state(state_index)
    if step_state is None:
        return False
    step_state = _state_to_device(step_state, env.device)
    n = len(env_ids)
    zero_vel = torch.zeros(n, 6, device=env.device)

    robot = env.scene["robot"]
    robot_pose = _expand_state_batch(step_state["articulation"]["robot"]["root_pose"], n)
    robot_pose[:, :3] += env.scene.env_origins[env_ids]
    robot.write_root_pose_to_sim(robot_pose, env_ids=env_ids)
    robot.write_root_velocity_to_sim(zero_vel, env_ids=env_ids)

    env.sim.forward()
    return True


class FFWSG2PickPlaceLTableMimicEnv(FFWSG2PickPlaceMimicEnv):
    """Mimic env for L-table: dual-arm IK; L-motion via recorded state replay in datagen."""

    def __init__(self, cfg: ManagerBasedRLEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        self._l_motion_ctrl = LTableKinematicLMotion(self)
        self._mimic_recorded_state: tuple[EpisodeData, int] | None = None
        self._mimic_carry_latch: bool = False
        self._kinematic_step_last_time: float | None = None
        self._last_step_was_kinematic: bool = False

    def reset(self, *args, **kwargs):
        self._mimic_body_joint_cmd = None
        self._mimic_recorded_state = None
        self._mimic_carry_latch = False
        self._kinematic_step_last_time = None
        self._last_step_was_kinematic = False
        self._l_motion_ctrl.reset()
        ret = super().reset(*args, **kwargs)
        self.sim.forward()
        return ret

    def step(self, action: torch.Tensor) -> VecEnvStepReturn:
        action = self._inject_mimic_body_joints(action)
        recorded = self._mimic_recorded_state
        carry_latch = self._mimic_carry_latch
        self._mimic_recorded_state = None
        self._mimic_carry_latch = False
        if recorded is not None:
            if not self._last_step_was_kinematic:
                self._kinematic_step_last_time = None
            episode, state_index = recorded
            self._last_step_was_kinematic = True
            return self._recorded_state_kinematic_step(action, episode, state_index, carry_latch=carry_latch)
        if self._last_step_was_kinematic:
            self._l_motion_ctrl.reset()
        self._last_step_was_kinematic = False
        return super(FFWSG2PickPlaceMimicEnv, self).step(action)

    def _recorded_state_kinematic_step(
        self, action: torch.Tensor, episode: EpisodeData, state_index: int, *, carry_latch: bool = False
    ) -> VecEnvStepReturn:
        """Recording-style step: robot base replay only when bimanual grasp is latched."""
        self.action_manager.process_action(action.to(self.device))
        env_ids = torch.arange(self.num_envs, device=self.device)

        self.scene.update(dt=self.physics_dt)
        if carry_latch:
            self._l_motion_ctrl.update_carry_latch(env_ids)
            if self._l_motion_ctrl.is_l_motion_latched(env_ids).any():
                apply_recorded_robot_root_state(self, episode, state_index, env_ids=env_ids)
                self.scene.update(dt=self.physics_dt)
                self._l_motion_ctrl.apply_latched_carry(env_ids)
        else:
            self._l_motion_ctrl.clear_carry(env_ids)

        self.recorder_manager.record_pre_step()
        is_rendering = self.sim.has_gui() or self.sim.has_rtx_sensors()

        self.episode_length_buf += 1
        self.common_step_counter += 1

        if len(self.recorder_manager.active_terms) > 0:
            self.obs_buf = self.observation_manager.compute()
            self.recorder_manager.record_post_step()

        self._pace_kinematic_step(is_rendering)

        return (
            self.obs_buf,
            self.reward_buf,
            self.reset_terminated,
            self.reset_time_outs,
            self.extras,
        )

    def _pace_kinematic_step(self, is_rendering: bool) -> None:
        """Hold one env step interval so L-motion matches VR recording / playback speed."""
        step_dt = float(self.step_dt)
        now = time.monotonic()
        if self._kinematic_step_last_time is not None:
            elapsed = now - self._kinematic_step_last_time
            remaining = step_dt - elapsed
            if remaining > 0.0:
                time.sleep(remaining)
        self._kinematic_step_last_time = time.monotonic()
        if is_rendering:
            self.sim.render()
