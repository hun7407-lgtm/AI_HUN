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

from dataclasses import MISSING

import isaaclab.sim as sim_utils
from isaaclab.envs.mdp.recorders.recorders_cfg import ActionStateRecorderManagerCfg as RecordTerm
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.controllers.differential_ik_cfg import DifferentialIKControllerCfg
from isaaclab.envs.mdp.actions.actions_cfg import DifferentialInverseKinematicsActionCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import FrameTransformerCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import GroundPlaneCfg
from isaaclab.utils import configclass
from isaaclab.sensors import CameraCfg

from . import mdp
from .mdp.ffw_sg2_l_table_events import LEFT_TABLE_DROP_HEIGHT_TOLERANCE, LEFT_TABLE_EDGE_MARGIN


@configclass
class LTableSceneCfg(InteractiveSceneCfg):
    """L-shaped table scene with a cardboard box on the front table."""

    robot: ArticulationCfg = MISSING
    left_eef: FrameTransformerCfg = MISSING
    right_eef: FrameTransformerCfg = MISSING

    table_front: AssetBaseCfg = MISSING
    table_left: AssetBaseCfg = MISSING
    cardboard_box: AssetBaseCfg = MISSING
    box_riser: AssetBaseCfg = MISSING
    drop_zone_marker: AssetBaseCfg | None = None
    # Real-robot parity cameras. ``cam_head`` is the ZED left eye (kept under its stock name
    # so the ~50 other references still resolve); the converter exports it as cam_left_head.
    cam_head: CameraCfg = MISSING
    cam_right_head: CameraCfg = MISSING
    cam_left_wrist: CameraCfg = MISSING
    cam_right_wrist: CameraCfg = MISSING

    plane = AssetBaseCfg(
        prim_path="/World/GroundPlane",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0, 0, 0.0]),
        spawn=GroundPlaneCfg(),
    )

    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )


@configclass
class ActionsCfg:
    arm_l_action: mdp.ActionTermCfg = MISSING
    gripper_l_action: mdp.ActionTermCfg = MISSING
    arm_r_action: mdp.ActionTermCfg = MISSING
    gripper_r_action: mdp.ActionTermCfg = MISSING
    lift_action: mdp.ActionTermCfg = MISSING
    head_action: mdp.ActionTermCfg = MISSING


@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        actions = ObsTerm(func=mdp.last_action)
        joint_pos = ObsTerm(
            func=mdp.joint_pos_name,
            params={
                "joint_names": [
                    "arm_l_joint1", "arm_l_joint2", "arm_l_joint3", "arm_l_joint4",
                    "arm_l_joint5", "arm_l_joint6", "arm_l_joint7", "gripper_l_joint1",
                    "arm_r_joint1", "arm_r_joint2", "arm_r_joint3", "arm_r_joint4",
                    "arm_r_joint5", "arm_r_joint6", "arm_r_joint7", "gripper_r_joint1",
                    "head_joint1", "lift_joint", "head_joint2",
                ],
                "asset_name": "robot",
            },
        )
        joint_pos_target = ObsTerm(
            func=mdp.joint_pos_target_name,
            params={
                "joint_names": [
                    "arm_l_joint1", "arm_l_joint2", "arm_l_joint3", "arm_l_joint4",
                    "arm_l_joint5", "arm_l_joint6", "arm_l_joint7", "gripper_l_joint1",
                    "arm_r_joint1", "arm_r_joint2", "arm_r_joint3", "arm_r_joint4",
                    "arm_r_joint5", "arm_r_joint6", "arm_r_joint7", "gripper_r_joint1",
                    "head_joint1", "lift_joint", "head_joint2",
                ],
                "asset_name": "robot",
            },
        )
        left_eef_pose = ObsTerm(
            func=mdp.eef_pose,
            params={"eef_cfg": SceneEntityCfg("left_eef"), "robot_cfg": SceneEntityCfg("robot")},
        )
        right_eef_pose = ObsTerm(
            func=mdp.eef_pose,
            params={"eef_cfg": SceneEntityCfg("right_eef"), "robot_cfg": SceneEntityCfg("robot")},
        )
        # Real-robot parity: ZED stereo head (left = cam_head) + a wrist camera per arm.
        # These ObsTerm names become the HDF5 obs keys the LeRobot converter reads.
        cam_head = ObsTerm(
            func=mdp.image,
            params={"sensor_cfg": SceneEntityCfg("cam_head"), "data_type": "rgb", "normalize": False},
        )
        cam_right_head = ObsTerm(
            func=mdp.image,
            params={"sensor_cfg": SceneEntityCfg("cam_right_head"), "data_type": "rgb", "normalize": False},
        )
        cam_left_wrist = ObsTerm(
            func=mdp.image,
            params={"sensor_cfg": SceneEntityCfg("cam_left_wrist"), "data_type": "rgb", "normalize": False},
        )
        cam_right_wrist = ObsTerm(
            func=mdp.image,
            params={"sensor_cfg": SceneEntityCfg("cam_right_wrist"), "data_type": "rgb", "normalize": False},
        )

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False

    @configclass
    class SubtaskCfg(ObsGroup):
        dual_grasp_box = ObsTerm(
            func=mdp.object_dual_grasped,
            params={
                "robot_cfg": SceneEntityCfg("robot"),
                "left_eef_cfg": SceneEntityCfg("left_eef"),
                "right_eef_cfg": SceneEntityCfg("right_eef"),
                "object_cfg": SceneEntityCfg("cardboard_box"),
            },
        )
        box_on_left_table = ObsTerm(
            func=mdp.object_on_left_table,
            params={
                "object_cfg": SceneEntityCfg("cardboard_box"),
                "table_left_cfg": SceneEntityCfg("table_left"),
                "edge_margin": LEFT_TABLE_EDGE_MARGIN,
                "height_tolerance": LEFT_TABLE_DROP_HEIGHT_TOLERANCE,
            },
        )

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False

    policy: PolicyCfg = PolicyCfg()
    subtask_terms: SubtaskCfg = SubtaskCfg()


@configclass
class TerminationsCfg:
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    success = DoneTerm(
        func=mdp.task_done,
        params={
            "object_cfg": SceneEntityCfg("cardboard_box"),
            "table_left_cfg": SceneEntityCfg("table_left"),
            "edge_margin": LEFT_TABLE_EDGE_MARGIN,
            "height_tolerance": LEFT_TABLE_DROP_HEIGHT_TOLERANCE,
        },
    )
    object_dropped = DoneTerm(
        func=mdp.object_dropped,
        params={
            "object_cfg": SceneEntityCfg("cardboard_box"),
            "velocity_threshold": 2.0,
        },
    )


@configclass
class PickPlaceLTableEnvCfg(ManagerBasedRLEnvCfg):
    """Dual-gripper box pick from front table and place on left table."""

    scene: LTableSceneCfg = LTableSceneCfg(num_envs=4096, env_spacing=3.0, replicate_physics=False)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    recorders: RecordTerm = RecordTerm()

    commands = None
    rewards = None
    events = None
    curriculum = None

    teleop_l_use_swerve: bool = False
    teleop_auto_l_on_grip_s: float = 2.0
    def __post_init__(self):
        self.decimation = 5
        self.episode_length_s = 45.0
        self.sim.dt = 0.01
        self.sim.render_interval = 2
        self.sim.physx.bounce_threshold_velocity = 0.01
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024 * 4
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 16 * 1024
        self.sim.physx.friction_correlation_distance = 0.00625

    def init_action_cfg(self, mode: str):
        if mode in ["record", "inference"]:
            self.actions.arm_l_action = mdp.JointPositionActionCfg(
                asset_name="robot",
                joint_names=["arm_l_joint[1-7]"],
                scale=1.0,
                use_default_offset=False,
            )
            self.actions.gripper_l_action = mdp.JointPositionActionCfg(
                asset_name="robot",
                joint_names=["gripper_l_joint1"],
                scale=1.0,
                use_default_offset=False,
            )
            self.actions.arm_r_action = mdp.JointPositionActionCfg(
                asset_name="robot",
                joint_names=["arm_r_joint[1-7]"],
                scale=1.0,
                use_default_offset=False,
            )
            self.actions.gripper_r_action = mdp.JointPositionActionCfg(
                asset_name="robot",
                joint_names=["gripper_r_joint1"],
                scale=1.0,
                use_default_offset=False,
            )
            self.actions.head_action = mdp.JointPositionActionCfg(
                asset_name="robot",
                joint_names=["head_joint1", "head_joint2"],
                scale=1.0,
            )
            self.actions.lift_action = mdp.JointPositionActionCfg(
                asset_name="robot",
                joint_names=["lift_joint"],
                scale=1.0,
            )
        elif mode in ["mimic_ik"]:
            self.actions.arm_l_action = DifferentialInverseKinematicsActionCfg(
                asset_name="robot",
                joint_names=["arm_l_joint[1-7]"],
                body_name="arm_l_link7",
                controller=DifferentialIKControllerCfg(
                    command_type="pose",
                    ik_params={"lambda_val": 0.05},
                    ik_method="dls",
                    use_relative_mode=False,
                ),
                body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(pos=[0.0, 0.0, -0.2]),
            )
            self.actions.gripper_l_action = mdp.JointPositionActionCfg(
                asset_name="robot",
                joint_names=["gripper_l_joint1"],
                scale=1.0,
                use_default_offset=False,
            )
            self.actions.arm_r_action = DifferentialInverseKinematicsActionCfg(
                asset_name="robot",
                joint_names=["arm_r_joint[1-7]"],
                body_name="arm_r_link7",
                controller=DifferentialIKControllerCfg(
                    command_type="pose",
                    ik_params={"lambda_val": 0.05},
                    ik_method="dls",
                    use_relative_mode=False,
                ),
                body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(pos=[0.0, 0.0, -0.2]),
            )
            self.actions.gripper_r_action = mdp.JointPositionActionCfg(
                asset_name="robot",
                joint_names=["gripper_r_joint1"],
                scale=1.0,
                use_default_offset=False,
            )
            self.actions.head_action = mdp.JointPositionActionCfg(
                asset_name="robot",
                joint_names=["head_joint1", "head_joint2"],
                scale=1.0,
                use_default_offset=False,
            )
            self.actions.lift_action = mdp.JointPositionActionCfg(
                asset_name="robot",
                joint_names=["lift_joint"],
                scale=1.0,
                use_default_offset=False,
            )
        else:
            raise ValueError(f"Unknown action mode: {mode}")
