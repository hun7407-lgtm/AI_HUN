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

"""Shared Mimic datagen wiring for dual-arm SG2 cardboard-box tasks."""

from __future__ import annotations

import math

from isaaclab.envs.mimic_env_cfg import (
    SubTaskConfig,
    SubTaskConstraintConfig,
    SubTaskConstraintCoordinationScheme,
    SubTaskConstraintType,
)


def _box_subtask_configs(*, place_signal: str, place_description: str) -> list[SubTaskConfig]:
    return [
        SubTaskConfig(
            object_ref="cardboard_box",
            subtask_term_signal="dual_grasp_box",
            subtask_term_offset_range=(10, 20),
            selection_strategy="nearest_neighbor_object",
            selection_strategy_kwargs={"nn_k": 1},
            action_noise=0.003,
            num_interpolation_steps=5,
            num_fixed_steps=0,
            apply_noise_during_interpolation=False,
            description="Dual grasp box",
            next_subtask_description=place_description,
        ),
        SubTaskConfig(
            object_ref="cardboard_box",
            subtask_term_signal=place_signal,
            subtask_term_offset_range=(5, 10),
            selection_strategy="nearest_neighbor_object",
            selection_strategy_kwargs={"nn_k": 1},
            action_noise=0.003,
            num_interpolation_steps=10,
            num_fixed_steps=0,
            apply_noise_during_interpolation=False,
            description=place_description,
            next_subtask_description="Task complete",
        ),
        SubTaskConfig(
            object_ref=None,
            subtask_term_signal=None,
            subtask_term_offset_range=(0, 0),
            selection_strategy="random",
            selection_strategy_kwargs={},
            action_noise=0.0001,
            num_interpolation_steps=5,
            num_fixed_steps=0,
            apply_noise_during_interpolation=False,
        ),
    ]


def _apply_datagen_defaults(cfg, *, datagen_name: str) -> None:
    cfg.datagen_config.name = datagen_name
    cfg.datagen_config.generation_guarantee = True
    cfg.datagen_config.generation_keep_failed = False
    cfg.datagen_config.generation_num_trials = 10
    cfg.datagen_config.generation_select_src_per_subtask = True
    cfg.datagen_config.generation_transform_first_robot_pose = False
    cfg.datagen_config.generation_interpolate_from_last_target_pose = True
    cfg.datagen_config.generation_relative = True
    cfg.datagen_config.max_num_failures = 25
    cfg.datagen_config.seed = 42


def configure_dual_arm_box_mimic(
    cfg,
    *,
    datagen_name: str,
    place_signal: str,
    place_description: str,
    arm_side: str = "right",
) -> None:
    """Attach standard grasp → place subtasks for single-EEF mimic (legacy)."""
    _apply_datagen_defaults(cfg, datagen_name=datagen_name)
    cfg.subtask_configs[f"{arm_side}_arm"] = _box_subtask_configs(
        place_signal=place_signal,
        place_description=place_description,
    )


def configure_ltable_dual_arm_mimic(
    cfg,
    *,
    datagen_name: str,
    place_signal: str,
    place_description: str,
) -> None:
    """Bimanual L-table mimic: both arms coordinated + scripted kinematic L-motion."""
    _apply_datagen_defaults(cfg, datagen_name=datagen_name)
    subtasks = _box_subtask_configs(place_signal=place_signal, place_description=place_description)
    # Skip place-subtask interpolation so L-motion state replay starts without a physics gap.
    subtasks[1].num_interpolation_steps = 0
    cfg.subtask_configs["left_arm"] = subtasks
    cfg.subtask_configs["right_arm"] = subtasks
    cfg.task_constraint_configs = [
        SubTaskConstraintConfig(
            eef_subtask_constraint_tuple=[("right_arm", 0), ("left_arm", 0)],
            constraint_type=SubTaskConstraintType.COORDINATION,
            coordination_scheme=SubTaskConstraintCoordinationScheme.REPLAY,
            coordination_synchronize_start=True,
        ),
        SubTaskConstraintConfig(
            eef_subtask_constraint_tuple=[("right_arm", 1), ("left_arm", 1)],
            constraint_type=SubTaskConstraintType.COORDINATION,
            coordination_scheme=SubTaskConstraintCoordinationScheme.REPLAY,
            coordination_synchronize_start=True,
        ),
    ]
    cfg.scripted_l_motion_enable = False
    cfg.teleop_l_yaw = math.pi / 2.0
    cfg.teleop_l_forward_m = 0.30
    cfg.teleop_l_rotation_duration_s = 3.0
    cfg.teleop_l_forward_duration_s = 2.0
    cfg.teleop_l_target_label = "left table"
