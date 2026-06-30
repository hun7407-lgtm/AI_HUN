# generated SH5 joint cfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

from cyclo_lab.real_world_tasks.manager_based.FFW_SG2.pick_place.mdp import ffw_sg2_pick_place_events
from cyclo_lab.real_world_tasks.manager_based.FFW_SG2.single_box_far.mdp import single_box_far_events
from cyclo_lab.real_world_tasks.manager_based.FFW_SH5._shared import attach_sh5_robot_and_sensors
from cyclo_lab.real_world_tasks.manager_based.FFW_SH5.single_box_far.single_box_far_sh5_env_cfg import SingleBoxFarSH5EnvCfg
from cyclo_lab.assets.object.table_prim import TABLE_FRONT_CFG, TABLE_LEFT_CFG
from cyclo_lab.assets.object.cardboard_box import CARDBOARD_BOX_CFG
from cyclo_lab.assets.object.box_riser import BOX_RISER_CFG


@configclass
class EventCfg:
    set_robot_joint_pose = EventTerm(
        func=ffw_sg2_pick_place_events.set_default_joint_pose,
        mode="reset",
        params={
            "joint_positions": {
                "arm_l_joint1": 0.75, "arm_l_joint4": -2.30,
                "arm_r_joint1": 0.75, "arm_r_joint4": -2.30,
                "head_joint1": 0.549, "lift_joint": -0.15,
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

    randomize_scene = EventTerm(
        func=single_box_far_events.randomize_single_box_far_scene,
        mode="reset",
        params={
            "table_front_cfg": SceneEntityCfg("table_front"),
            "table_rear_cfg": SceneEntityCfg("table_rear"),
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
class FFWSH5SingleBoxFarEnvCfg(SingleBoxFarSH5EnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.events = EventCfg()

        attach_sh5_robot_and_sensors(self.scene)
        self.scene.table_front = TABLE_FRONT_CFG.replace(prim_path="{ENV_REGEX_NS}/TableFront")
        self.scene.table_rear = TABLE_LEFT_CFG.replace(prim_path="{ENV_REGEX_NS}/TableRear")
        self.scene.cardboard_box = CARDBOARD_BOX_CFG.replace(prim_path="{ENV_REGEX_NS}/CardboardBox")
        self.scene.box_riser = BOX_RISER_CFG.replace(prim_path="{ENV_REGEX_NS}/BoxRiser")
