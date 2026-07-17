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

import isaaclab.sim as sim_utils
from isaaclab.envs import mdp as isaac_mdp
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import FrameTransformerCfg, CameraCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from isaaclab.utils import configclass

from cyclo_lab.real_world_tasks.manager_based.FFW_SG2.pick_place.mdp import ffw_sg2_pick_place_events
from cyclo_lab.real_world_tasks.manager_based.FFW_SG2.pick_place_l_table.mdp import ffw_sg2_l_table_events
from cyclo_lab.real_world_tasks.manager_based.FFW_SG2.pick_place_l_table.pick_place_env_cfg import PickPlaceLTableEnvCfg

from isaaclab.markers.config import FRAME_MARKER_CFG  # isort: skip
from cyclo_lab.assets.robots.FFW_SG2 import FFW_SG2_CFG  # isort: skip
from cyclo_lab.assets.robots.FFW_SG2_MOBILE import FFW_SG2_MOBILE_CFG  # isort: skip
from cyclo_lab.assets.object.table_prim import TABLE_FRONT_CFG, TABLE_LEFT_CFG
from cyclo_lab.assets.object.cardboard_box import CARDBOARD_BOX_CFG
from cyclo_lab.assets.object.box_riser import BOX_RISER_CFG
from cyclo_lab.assets.object.drop_zone_marker import L_TABLE_DROP_ZONE_MARKER_CFG


@configclass
class EventCfg:
    reset_scene_to_default = EventTerm(
        func=isaac_mdp.reset_scene_to_default,
        mode="reset",
    )

    set_robot_joint_pose = EventTerm(
        func=ffw_sg2_pick_place_events.set_default_joint_pose,
        mode="reset",
        params={
            "joint_positions": {
                "arm_l_joint1": 0.75, "arm_l_joint4": -2.30,
                "arm_r_joint1": 0.75, "arm_r_joint4": -2.30,
                "head_joint1": 0.549, "lift_joint": 0.0365,
            },
            "asset_cfg": SceneEntityCfg("robot"),
        },
    )

    randomize_ffw_sg2_joint_state = EventTerm(
        func=ffw_sg2_pick_place_events.randomize_joint_by_gaussian_offset,
        mode="reset",
        params={
            "mean": 0.0,
            "std": 0.05,
            "joint_names": [
                "arm_l_joint1", "arm_l_joint2", "arm_l_joint3", "arm_l_joint4",
                "arm_l_joint5", "arm_l_joint6", "arm_l_joint7",
                "arm_r_joint1", "arm_r_joint2", "arm_r_joint3", "arm_r_joint4",
                "arm_r_joint5", "arm_r_joint6", "arm_r_joint7",
                "head_joint1", "head_joint2",
            ],
            "asset_cfg": SceneEntityCfg("robot"),
        },
    )

    randomize_head_camera = EventTerm(
        func=ffw_sg2_pick_place_events.randomize_camera_pose,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("cam_head"),
            "pose_range": {
                "x": (0.0, 0.01),
                "y": (-0.01, 0.01),
                "z": (-0.01, 0.01),
                "roll": (-0.01, 0.01),
                "pitch": (-0.01, 0.01),
                "yaw": (-0.01, 0.01),
            },
            "convention": "ros",
        },
    )

    randomize_l_table_scene = EventTerm(
        func=ffw_sg2_l_table_events.randomize_l_table_scene,
        mode="reset",
        params={
            "table_front_cfg": SceneEntityCfg("table_front"),
            "table_left_cfg": SceneEntityCfg("table_left"),
            "box_cfg": SceneEntityCfg("cardboard_box"),
            "box_riser_cfg": SceneEntityCfg("box_riser"),
            "box_pose_range": {
                "x": (-0.04, 0.04),
                "y": (-0.06, 0.06),
                "yaw": (-0.1, 0.1),
            },
        },
    )

    randomize_scene_light = EventTerm(
        func=ffw_sg2_pick_place_events.randomize_scene_lighting_domelight,
        mode="reset",
        params={
            "intensity_range": (500.0, 3000.0),
            "color_range": ((0.8, 1.0), (0.8, 1.0), (0.8, 1.0)),
            "asset_cfg": SceneEntityCfg("light"),
        },
    )


@configclass
class FFWSG2PickPlaceLTableEnvCfg(PickPlaceLTableEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.events = EventCfg()

        self.scene.robot = FFW_SG2_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.robot.spawn.semantic_tags = [("class", "robot")]

        self.scene.table_front = TABLE_FRONT_CFG.replace(prim_path="{ENV_REGEX_NS}/TableFront")
        self.scene.table_left = TABLE_LEFT_CFG.replace(prim_path="{ENV_REGEX_NS}/TableLeft")
        self.scene.cardboard_box = CARDBOARD_BOX_CFG.replace(prim_path="{ENV_REGEX_NS}/CardboardBox")
        self.scene.box_riser = BOX_RISER_CFG.replace(prim_path="{ENV_REGEX_NS}/BoxRiser")
        self.scene.drop_zone_marker = L_TABLE_DROP_ZONE_MARKER_CFG.replace(
            prim_path="{ENV_REGEX_NS}/DropZoneMarker"
        )
        self.scene.plane.semantic_tags = [("class", "ground")]

        # --- Real-robot parity cameras --------------------------------------------------
        # Real ffw_sg2_rev1 records four RGB streams (matching lift_box_merged_lerobot_*):
        #   cam_left_head / cam_right_head  = ZED stereo eyes on the head  (376 x 672)
        #   cam_left_wrist / cam_right_wrist = one camera per gripper wrist ( 424 x 240 )
        # The stock task rendered only the head left eye. The three cameras below add the
        # rest so recordings match the real observation schema.
        #
        # CALIBRATE: the offsets/intrinsics here are reasonable placeholders, NOT measured
        # against the physical rig. Tune STEREO_BASELINE and the wrist offsets by eye against
        # the real cam_*_wrist / cam_right_head images before trusting cross-domain training.
        _ZED_HEAD_SPAWN = sim_utils.PinholeCameraCfg(
            focal_length=10.4,
            focus_distance=200.0,
            horizontal_aperture=20.955,
            clipping_range=(0.01, 100.0),
        )
        # The `head_link2/zed` prim carries no explicit eye prims (measured: it is a single
        # prim centred at y=0 on head_link2). So place the two eyes symmetric about that
        # centre in Y. The stock cam_head used +0.03, so treat that as the half-baseline:
        # left eye at +HALF, right eye at -HALF -> ~0.06 m baseline (ZED Mini class).
        STEREO_HALF = 0.03

        self.scene.cam_head = CameraCfg(
            prim_path="{ENV_REGEX_NS}/Robot/ffw_sg2_follower/head_link2/zed/cam_head",
            update_period=0.0,
            height=376,
            width=672,
            data_types=["rgb"],
            spawn=_ZED_HEAD_SPAWN.copy(),
            offset=CameraCfg.OffsetCfg(
                pos=(0.0, STEREO_HALF, 0.0),  # left eye
                rot=(0.5, 0.5, -0.5, -0.5),
                convention="isaac",
            ),
        )
        self.scene.cam_right_head = CameraCfg(
            prim_path="{ENV_REGEX_NS}/Robot/ffw_sg2_follower/head_link2/zed/cam_right_head",
            update_period=0.0,
            height=376,
            width=672,
            data_types=["rgb"],
            spawn=_ZED_HEAD_SPAWN.copy(),
            offset=CameraCfg.OffsetCfg(
                pos=(0.0, -STEREO_HALF, 0.0),  # right eye, mirrored about the zed centre
                rot=(0.5, 0.5, -0.5, -0.5),
                convention="isaac",
            ),
        )

        _WRIST_SPAWN = sim_utils.PinholeCameraCfg(
            focal_length=8.0,
            focus_distance=200.0,
            horizontal_aperture=20.955,
            clipping_range=(0.01, 100.0),
        )
        # Portrait 424 x 240 to match the real wrist streams. Mounted at the RealSense D405
        # pose measured from the USD (the ``arm_*_link7/visuals/d405`` prim): the real wrist
        # camera location. Left and right share the same local transform.
        #   pos  = (0.10683, 0.0, -0.07713)   relative to arm_*_link7
        #   rot  = 180 deg about Y  ->  quat (w,x,y,z) = (0, 0, 1, 0)  (d405 body frame)
        # convention="ros" applies the RealSense optical-frame convention on top of that body
        # orientation. If the view still faces the wrong way, adjust rot/convention here.
        _WRIST_POS = (0.10683, 0.0, -0.07713)
        _WRIST_ROT = (0.0, 0.0, 1.0, 0.0)
        self.scene.cam_left_wrist = CameraCfg(
            prim_path="{ENV_REGEX_NS}/Robot/ffw_sg2_follower/arm_l_link7/cam_left_wrist",
            update_period=0.0,
            height=424,
            width=240,
            data_types=["rgb"],
            spawn=_WRIST_SPAWN.copy(),
            offset=CameraCfg.OffsetCfg(pos=_WRIST_POS, rot=_WRIST_ROT, convention="ros"),
        )
        self.scene.cam_right_wrist = CameraCfg(
            prim_path="{ENV_REGEX_NS}/Robot/ffw_sg2_follower/arm_r_link7/cam_right_wrist",
            update_period=0.0,
            height=424,
            width=240,
            data_types=["rgb"],
            spawn=_WRIST_SPAWN.copy(),
            offset=CameraCfg.OffsetCfg(pos=_WRIST_POS, rot=_WRIST_ROT, convention="ros"),
        )

        marker_cfg = FRAME_MARKER_CFG.copy()
        marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
        marker_cfg.prim_path = "/Visuals/FrameTransformer"

        self.scene.right_eef = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/ffw_sg2_follower/arm_base_link",
            debug_vis=False,
            visualizer_cfg=marker_cfg,
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/ffw_sg2_follower/arm_r_link7",
                    name="end_effector",
                    offset=OffsetCfg(pos=[0.0, 0.0, -0.2]),
                ),
            ],
        )

        self.scene.left_eef = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/ffw_sg2_follower/arm_base_link",
            debug_vis=False,
            visualizer_cfg=marker_cfg,
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/ffw_sg2_follower/arm_l_link7",
                    name="end_effector",
                    offset=OffsetCfg(pos=[0.0, 0.0, -0.2]),
                ),
            ],
        )


@configclass
class FFWSG2PickPlaceLTableMobileEnvCfg(FFWSG2PickPlaceLTableEnvCfg):
    """Drivable-base L-table variant (Plan B).

    The operator drives the swerve base physically from ``/cmd_vel`` (VR thumbstick
    translation + A/B rotation) during recording instead of the scripted L-motion. Uses the
    FFW_SG2_MOBILE robot (free base + live wheels). Recorded base velocity gives real-robot
    parity (linear_x/y + angular_z). The stock L-table task is left untouched.
    """

    def __post_init__(self):
        super().__post_init__()
        # Swap the welded stock base for the drivable FFW_SG2_MOBILE (free base + wheels).
        self.scene.robot = FFW_SG2_MOBILE_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.robot.spawn.semantic_tags = [("class", "robot")]
        # Free base driving from cmd_vel; disable the scripted L-motion / auto-L teleport.
        self.teleop_base_drive = True
        self.teleop_auto_l_on_grip_s = 0.0
