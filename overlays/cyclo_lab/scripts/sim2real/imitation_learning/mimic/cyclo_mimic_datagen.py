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

"""Cyclo extensions for Isaac Lab Mimic datagen (lift/head + state-aware L-motion replay)."""

from __future__ import annotations

import asyncio
import inspect

import torch

import isaaclab.utils.math as math_utils

from isaaclab.envs import ManagerBasedRLMimicEnv
from isaaclab.managers import TerminationTermCfg
from isaaclab.utils.datasets import EpisodeData

from isaaclab_mimic.datagen.data_generator import DataGenerator
from isaaclab_mimic.datagen.datagen_info_pool import DataGenInfoPool
from isaaclab_mimic.datagen.waypoint import MultiWaypoint, Waypoint, WaypointSequence, WaypointTrajectory

# Record / replay layout (ActionManager order): lift@16, head1@17, head2@18.
_ACT_BODY = slice(16, 19)

# Fallback when recorded lift channel is flat in both actions and obs.
_SCRIPT_LIFT_START = 0.0365
_SCRIPT_LIFT_END = -0.0993
_SCRIPT_LIFT_RAMP_STEPS = 40
_SCRIPT_LIFT_MIN_STD = 1e-3


def body_joint_cmds_from_actions(actions: torch.Tensor) -> torch.Tensor:
    """Return per-step [lift, head1, head2] from recorded actions (record replay layout)."""
    return actions[:, _ACT_BODY].clone()


def body_joint_cmds_from_joint_pos(joint_pos: torch.Tensor) -> torch.Tensor:
    """Fallback: map obs joint layout (head1, lift, head2) to action layout (lift, head)."""
    lift = joint_pos[:, 17:18]
    head = joint_pos[:, [16, 18]]
    return torch.cat([lift, head], dim=1)


def _script_lift_approach(body_joint_cmds: torch.Tensor) -> torch.Tensor:
    """Ramp torso lift down at grasp approach (VR lift was rarely stored per-step)."""
    if body_joint_cmds.shape[0] == 0:
        return body_joint_cmds
    out = body_joint_cmds.clone()
    demo_min = float(out[:, 0].min().item())
    end_lift = min(_SCRIPT_LIFT_END, demo_min)
    ramp = min(_SCRIPT_LIFT_RAMP_STEPS, out.shape[0])
    for i in range(ramp):
        alpha = (i + 1) / ramp
        out[i, 0] = _SCRIPT_LIFT_START + alpha * (end_lift - _SCRIPT_LIFT_START)
    return out


def _state_to_device(state: dict, device: torch.device) -> dict:
    """Move a nested scene-state dict onto the simulation device."""
    output: dict = {}
    for asset_type, assets in state.items():
        output[asset_type] = {}
        for asset_name, fields in assets.items():
            output[asset_type][asset_name] = {
                key: value.to(device) if isinstance(value, torch.Tensor) else value
                for key, value in fields.items()
            }
    return output


def _episode_has_step_states(episode: EpisodeData) -> bool:
    states = episode.data.get("states")
    return isinstance(states, dict) and len(states) > 0


def apply_recorded_step_state(
    env: ManagerBasedRLMimicEnv,
    episode: EpisodeData,
    state_index: int,
    env_ids: torch.Tensor | None = None,
) -> bool:
    """Apply a recorded post-step scene state — same helper used by demo playback."""
    step_state = episode.get_state(state_index)
    if step_state is None:
        return False
    step_state = _state_to_device(step_state, env.device)
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=env.device)
    env.scene.reset_to(step_state, env_ids=env_ids, is_relative=True)
    env.sim.forward()
    return True


class CycloWaypoint(Waypoint):
    """Waypoint with lift/head targets replayed from source demonstrations."""

    def __init__(self, pose, gripper_action, body_joint_cmd=None, noise=None):
        super().__init__(pose, gripper_action, noise)
        self.body_joint_cmd = body_joint_cmd


class CycloDataGenInfoPool(DataGenInfoPool):
    """Datagen pool that retains source episodes for state-aware L-motion replay."""

    def __init__(self, env, env_cfg, device, asyncio_lock: asyncio.Lock | None = None):
        super().__init__(env, env_cfg, device, asyncio_lock)
        self.episode_body_joints: list[torch.Tensor] = []
        self.episodes: list[EpisodeData] = []

    def _add_episode(self, episode):
        super()._add_episode(episode)
        self.episodes.append(episode)

        if "actions" in episode.data:
            actions = episode.data["actions"]
            if actions.shape[1] < 19:
                raise ValueError(
                    f"Episode actions need >=19 dims for lift/head replay, got {actions.shape[1]}"
                )
            self.episode_body_joints.append(body_joint_cmds_from_actions(actions))
            return

        obs = episode.data["obs"]
        if "joint_pos_target" in obs:
            joint_data = obs["joint_pos_target"]
        elif "joint_pos" in obs:
            joint_data = obs["joint_pos"]
        else:
            raise ValueError("Episode lacks actions or obs/joint_pos(_target) for VR lift replay")
        self.episode_body_joints.append(body_joint_cmds_from_joint_pos(joint_data))


def _spread_waypoint_cmds(waypoints: list, per_step_cmds, attr: str) -> None:
    """Map stored per-step commands onto merged waypoint lists (incl. interpolation prefix)."""
    if not waypoints or per_step_cmds is None:
        return
    if isinstance(per_step_cmds, list) and len(per_step_cmds) == 0:
        return
    if isinstance(per_step_cmds, torch.Tensor) and per_step_cmds.shape[0] == 0:
        return

    n = len(waypoints)
    m = len(per_step_cmds) if isinstance(per_step_cmds, list) else per_step_cmds.shape[0]
    interp_len = max(0, n - m)
    for i, wp in enumerate(waypoints):
        src_i = 0 if i < interp_len else min(i - interp_len, m - 1)
        value = per_step_cmds[src_i] if isinstance(per_step_cmds, list) else per_step_cmds[src_i].clone()
        setattr(wp, attr, value)


def _spread_body_joint_cmds(waypoints: list, body_joint_cmds: torch.Tensor) -> None:
    _spread_waypoint_cmds(waypoints, body_joint_cmds, "body_joint_cmd")


def _episode_actions_tensor(episode: EpisodeData) -> torch.Tensor | None:
    actions = episode.data.get("actions")
    if actions is None:
        return None
    if isinstance(actions, torch.Tensor):
        return actions
    if isinstance(actions, list):
        if not actions:
            return None
        if isinstance(actions[0], torch.Tensor):
            return torch.stack(actions)
        return torch.tensor(actions)
    return torch.as_tensor(actions)


def _find_l_motion_end_index(episode: EpisodeData, start: int, end: int) -> int:
    """Inclusive last frame where the robot base is still moving (rotate + drive only)."""
    last_move = start
    prev_xy: torch.Tensor | None = None
    prev_yaw: float | None = None
    for i in range(start, end):
        step_state = episode.get_state(i)
        if step_state is None:
            continue
        root = step_state["articulation"]["robot"]["root_pose"]
        xy = root[0, :2]
        _, _, yaw = math_utils.euler_xyz_from_quat(root[:, 3:7])
        yaw_f = float(yaw[0].item())
        if prev_xy is not None:
            dxy = float(torch.linalg.vector_norm(xy - prev_xy).item())
            dyaw = abs(yaw_f - prev_yaw)
            if dxy > 5e-4 or dyaw > 0.005:
                last_move = i
        prev_xy = xy.clone()
        prev_yaw = yaw_f
    return last_move


def _find_kinematic_carry_end_index(episode: EpisodeData, start: int, end: int) -> int:
    """Last frame for kinematic base replay + rigid carry (through lower, until gripper release)."""
    base_end = _find_l_motion_end_index(episode, start, end)
    actions = _episode_actions_tensor(episode)
    if actions is None or actions.shape[1] < 16:
        return base_end

    last_carry = base_end
    for i in range(base_end, min(end, actions.shape[0])):
        if float(actions[i, 7].item()) > 0.2 and float(actions[i, 15].item()) > 0.2:
            last_carry = i
        else:
            break
    return last_carry


def _spread_recorded_state_indices_l_motion(
    waypoints: list,
    episode: EpisodeData,
    state_indices: list[int],
    *,
    body_joint_cmd_count: int,
) -> None:
    """Attach recorded states after the mimic interpolation prefix (aligned with body-joint spread)."""
    interp_len = max(0, len(waypoints) - body_joint_cmd_count)
    for i, wp in enumerate(waypoints):
        src_i = i - interp_len
        if 0 <= src_i < len(state_indices):
            wp.recorded_state_index = state_indices[src_i]
            wp.recorded_episode = episode
            wp.recorded_carry_latch = True
        else:
            wp.recorded_state_index = None
            wp.recorded_episode = None
            wp.recorded_carry_latch = False


def _spread_recorded_state_indices_prefix(
    waypoints: list, episode: EpisodeData, state_indices: list[int]
) -> None:
    """Attach recorded states to the L-motion prefix; place/release runs on physics."""
    for i, wp in enumerate(waypoints):
        if i < len(state_indices):
            wp.recorded_state_index = state_indices[i]
            wp.recorded_episode = episode
        else:
            wp.recorded_state_index = None
            wp.recorded_episode = None


def _spread_recorded_state_indices(
    waypoints: list, episode: EpisodeData, state_indices: list[int]
) -> None:
    """Attach one recorded state per source frame (skip mimic interpolation prefix)."""
    if not waypoints or not state_indices:
        return
    n = len(waypoints)
    m = len(state_indices)
    interp_len = max(0, n - m)
    for i, wp in enumerate(waypoints):
        if i < interp_len:
            wp.recorded_state_index = None
            wp.recorded_episode = None
        else:
            src_i = min(i - interp_len, m - 1)
            wp.recorded_state_index = state_indices[src_i]
            wp.recorded_episode = episode


def _attach_waypoint_cmds(traj: WaypointTrajectory, per_step_cmds, attr: str) -> None:
    idx = 0
    for sequence in traj.waypoint_sequences:
        for waypoint in sequence.sequence:
            if idx < (len(per_step_cmds) if isinstance(per_step_cmds, list) else per_step_cmds.shape[0]):
                setattr(waypoint, attr, per_step_cmds[idx])
            idx += 1


def _attach_body_joint_cmds(traj: WaypointTrajectory, body_joint_cmds: torch.Tensor) -> None:
    _attach_waypoint_cmds(traj, body_joint_cmds, "body_joint_cmd")


def _attach_recorded_state_indices(
    traj: WaypointTrajectory, episode: EpisodeData, state_indices: list[int]
) -> None:
    _attach_waypoint_cmds(traj, state_indices, "recorded_state_index")
    for sequence in traj.waypoint_sequences:
        for waypoint in sequence.sequence:
            waypoint.recorded_episode = episode


class CycloDataGenerator(DataGenerator):
    """Replay lift/head from actions and L-motion via per-step recorded states (like playback)."""

    def generate_eef_subtask_trajectory(
        self,
        env_id: int,
        eef_name: str,
        subtask_ind: int,
        all_randomized_subtask_boundaries: dict,
        runtime_subtask_constraints_dict: dict,
        selected_src_demo_inds: dict,
    ) -> WaypointTrajectory:
        traj = super().generate_eef_subtask_trajectory(
            env_id,
            eef_name,
            subtask_ind,
            all_randomized_subtask_boundaries,
            runtime_subtask_constraints_dict,
            selected_src_demo_inds,
        )

        pool = self.src_demo_datagen_info_pool
        if not isinstance(pool, CycloDataGenInfoPool) or not pool.episode_body_joints:
            return traj

        is_first_subtask = subtask_ind == 0
        selected_src_demo_ind = selected_src_demo_inds[eef_name]
        selected_src_subtask_boundary = all_randomized_subtask_boundaries[eef_name][selected_src_demo_ind][subtask_ind]
        start, end = selected_src_subtask_boundary

        body_joint_cmds = pool.episode_body_joints[selected_src_demo_ind][start:end].clone()
        if is_first_subtask or self.env_cfg.datagen_config.generation_transform_first_robot_pose:
            body_joint_cmds = torch.cat([body_joint_cmds[0:1], body_joint_cmds], dim=0)
        if subtask_ind == 0 and float(body_joint_cmds[:, 0].std().item()) < _SCRIPT_LIFT_MIN_STD:
            body_joint_cmds = _script_lift_approach(body_joint_cmds)

        traj._body_joint_cmds = body_joint_cmds
        _attach_body_joint_cmds(traj, body_joint_cmds)

        if subtask_ind == 0 and pool.episodes and eef_name == "left_arm":
            episode = pool.episodes[selected_src_demo_ind]
            if _episode_has_step_states(episode):
                boundaries = all_randomized_subtask_boundaries[eef_name][selected_src_demo_ind]
                if len(boundaries) > 1:
                    place_start = boundaries[1][0]
                    traj._pre_l_motion_state = (episode, place_start)

        # Subtask 1: kinematic replay for L-motion only; place/release uses physics.
        if subtask_ind == 1 and pool.episodes:
            episode = pool.episodes[selected_src_demo_ind]
            if _episode_has_step_states(episode):
                carry_end = _find_kinematic_carry_end_index(episode, start, end)
                state_indices = list(range(start, carry_end + 1))
                traj._recorded_episode = episode
                traj._recorded_state_indices = state_indices
                traj._recorded_state_prefix = True
                _attach_recorded_state_indices(traj, episode, state_indices)

        return traj

    def merge_eef_subtask_trajectory(
        self,
        env_id: int,
        eef_name: str,
        subtask_index: int,
        prev_executed_traj: list[Waypoint] | None,
        subtask_trajectory: WaypointTrajectory,
    ) -> list[Waypoint]:
        waypoints = super().merge_eef_subtask_trajectory(
            env_id, eef_name, subtask_index, prev_executed_traj, subtask_trajectory
        )
        _spread_body_joint_cmds(waypoints, getattr(subtask_trajectory, "_body_joint_cmds", None))
        pre_state = getattr(subtask_trajectory, "_pre_l_motion_state", None)
        if pre_state is not None:
            episode, state_index = pre_state
            for wp in waypoints[-5:]:
                wp.recorded_state_index = state_index
                wp.recorded_episode = episode
                wp.recorded_carry_latch = False
        episode = getattr(subtask_trajectory, "_recorded_episode", None)
        state_indices = getattr(subtask_trajectory, "_recorded_state_indices", None)
        if episode is not None and state_indices is not None:
            if getattr(subtask_trajectory, "_recorded_state_prefix", False):
                body_cmds = getattr(subtask_trajectory, "_body_joint_cmds", None)
                body_count = (
                    len(body_cmds)
                    if isinstance(body_cmds, list)
                    else (body_cmds.shape[0] if body_cmds is not None else 0)
                )
                _spread_recorded_state_indices_l_motion(
                    waypoints, episode, state_indices, body_joint_cmd_count=body_count
                )
            else:
                _spread_recorded_state_indices(waypoints, episode, state_indices)
        return waypoints


async def cyclo_multi_waypoint_execute(
    self: MultiWaypoint,
    env: ManagerBasedRLMimicEnv,
    success_term: TerminationTermCfg,
    env_id: int = 0,
    env_action_queue: asyncio.Queue | None = None,
):
    """Inject lift/head replay; restore recorded scene state after step (playback parity)."""
    body_cmd = None
    recorded_episode = None
    recorded_state_index = None
    recorded_carry_latch = False
    for waypoint in self.waypoints.values():
        if body_cmd is None:
            cmd = getattr(waypoint, "body_joint_cmd", None)
            if cmd is not None:
                body_cmd = cmd
        if recorded_state_index is None:
            idx = getattr(waypoint, "recorded_state_index", None)
            if idx is not None:
                recorded_state_index = idx
                recorded_episode = getattr(waypoint, "recorded_episode", None)
        if getattr(waypoint, "recorded_carry_latch", False):
            recorded_carry_latch = True

    if body_cmd is not None:
        env._mimic_body_joint_cmd = body_cmd.to(env.device)

    if recorded_episode is not None and recorded_state_index is not None:
        if hasattr(env, "_mimic_recorded_state"):
            env._mimic_recorded_state = (recorded_episode, recorded_state_index)
        if hasattr(env, "_mimic_carry_latch"):
            env._mimic_carry_latch = recorded_carry_latch

    if "action_noise_dict" in inspect.signature(env.target_eef_pose_to_action).parameters:
        action_noise_dict = {eef_name: waypoint.noise for eef_name, waypoint in self.waypoints.items()}
        play_action = env.target_eef_pose_to_action(
            target_eef_pose_dict={eef_name: waypoint.pose for eef_name, waypoint in self.waypoints.items()},
            gripper_action_dict={eef_name: waypoint.gripper_action for eef_name, waypoint in self.waypoints.items()},
            action_noise_dict=action_noise_dict,
            env_id=env_id,
        )
    else:
        play_action = env.target_eef_pose_to_action(
            target_eef_pose_dict={eef_name: waypoint.pose for eef_name, waypoint in self.waypoints.items()},
            gripper_action_dict={eef_name: waypoint.gripper_action for eef_name, waypoint in self.waypoints.items()},
            noise=max([waypoint.noise for waypoint in self.waypoints.values()]),
            env_id=env_id,
        )

    if play_action.dim() == 1:
        play_action = play_action.unsqueeze(0)

    play_action = env._inject_mimic_body_joints(play_action)

    if env_action_queue is None:
        obs, _, _, _, _ = env.step(play_action)
    else:
        await env_action_queue.put((env_id, play_action[0]))
        await env_action_queue.join()
        obs = env.obs_buf

    success = bool(success_term.func(env, **success_term.params)[env_id])

    state = env.scene.get_state(is_relative=True)
    return dict(
        states=[state],
        observations=[obs],
        actions=[play_action],
        success=success,
    )


def enable_cyclo_body_joint_replay() -> None:
    MultiWaypoint.execute = cyclo_multi_waypoint_execute


def setup_cyclo_async_generation(
    env,
    num_envs: int,
    input_file: str,
    success_term,
    pause_subtask: bool = False,
    motion_planners=None,
) -> dict:
    from isaaclab_mimic.datagen.generation import run_data_generator

    asyncio_event_loop = asyncio.get_event_loop()
    env_reset_queue = asyncio.Queue()
    env_action_queue = asyncio.Queue()
    shared_datagen_info_pool_lock = asyncio.Lock()
    shared_datagen_info_pool = CycloDataGenInfoPool(
        env, env.cfg, env.device, asyncio_lock=shared_datagen_info_pool_lock
    )
    shared_datagen_info_pool.load_from_dataset_file(input_file)
    print(
        f"Loaded {shared_datagen_info_pool.num_datagen_infos} to datagen info pool "
        "(lift/head from actions, L-motion from recorded states)"
    )

    data_generator = CycloDataGenerator(env=env, src_demo_datagen_info_pool=shared_datagen_info_pool)
    data_generator_asyncio_tasks = []
    for i in range(num_envs):
        env_motion_planner = motion_planners[i] if motion_planners else None
        task = asyncio_event_loop.create_task(
            run_data_generator(
                env,
                i,
                env_reset_queue,
                env_action_queue,
                data_generator,
                success_term,
                pause_subtask=pause_subtask,
                motion_planner=env_motion_planner,
            )
        )
        data_generator_asyncio_tasks.append(task)

    return {
        "tasks": data_generator_asyncio_tasks,
        "event_loop": asyncio_event_loop,
        "reset_queue": env_reset_queue,
        "action_queue": env_action_queue,
        "info_pool": shared_datagen_info_pool,
    }
