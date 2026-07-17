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
#
# Author: Taehyeong Kim

import json
import os
import math
import threading
import torch
import cv2
import time
from pynput.keyboard import Listener
from collections.abc import Callable
from datetime import datetime

import isaaclab.utils.math as math_utils

from robotis_dds_python.idl.trajectory_msgs.msg import JointTrajectory_
from robotis_dds_python.idl.sensor_msgs.msg import JointState_
from robotis_dds_python.idl.sensor_msgs.msg import CompressedImage_
from robotis_dds_python.idl.geometry_msgs.msg import Twist_
from robotis_dds_python.idl.std_msgs.msg import Header_
from robotis_dds_python.idl.std_msgs.msg import String_
from robotis_dds_python.idl.builtin_interfaces.msg import Time_

from robotis_dds_python.tools.topic_manager import TopicManager


SG2_POLICY_JOINT_NAMES = (
    [f"arm_l_joint{i}" for i in range(1, 8)]
    + ["gripper_l_joint1"]
    + [f"arm_r_joint{i}" for i in range(1, 8)]
    + ["gripper_r_joint1", "lift_joint", "head_joint1", "head_joint2"]
)

_LEFT_GRIP_SMOOTH_ALPHA = 0.20
_LEFT_FINGER_HOLD_MIN = 0.12
_LEFT_FINGER_HOLD_ALPHA = 0.38
_LEFT_FINGER_HOLD_ALPHA_FIRM = 0.62


class FFWSG2Sdk:
    """FFWSG2Sdk class for DDS teleoperation and publishing humanoid robot state/images."""

    # Subclasses (e.g. FFWSH5Sdk) override with BestEffort to match cyclo_motion_controller.
    TRAJECTORY_QOS = None

    def __init__(self, env, mode: str):
        self.env = env
        self.mode = mode  # 'record' or 'inference'
        self.running = True
        self.domain_id = int(os.getenv("ROS_DOMAIN_ID", 0))
        self.left_arm_trajectory_cmd = None
        self.right_arm_trajectory_cmd = None
        self._left_gripper_cmd: float | None = None
        self._right_gripper_cmd: float | None = None
        self._left_gripper_smooth: float | None = None
        self.head_joint_trajectory_cmd = None
        self.lift_joint_trajectory_cmd = None
        self._started = False
        self._reset_state = False
        self._additional_callbacks = {}
        self._first_episode = True  # Track if this is the first episode
        self._episode_phase = "idle"  # Current state: "idle" (waiting) or "recording" (active episode)
        self._face_left_yaw = math.pi / 2.0
        self._l_motion_label = "left table"
        self._pending_face_left = False
        self._rotation_active = False
        self._rotation_start_yaw = 0.0
        self._rotation_target_yaw = 0.0
        self._rotation_start_time = 0.0
        self._rotation_duration_s = 3.0
        self._rotation_sim_alpha = 0.0
        self._forward_active = False
        self._forward_start_pos = None
        self._forward_end_pos = None
        self._forward_yaw = 0.0
        self._forward_start_time = 0.0
        self._forward_duration_s = 2.0
        self._forward_sim_alpha = 0.0
        self._forward_distance_m = 0.30
        self._pending_reset_pose = False
        self._home_root_pose = None  # (1, 7) tensor captured on first publish
        # While the L (face-left) motion teleports the robot root, the box is
        # carried rigidly with it via this relative transform so it does not
        # slip out of the grippers.
        self._carry_box = False
        self._box_rel_pos = None  # (1, 3) box pos in robot root frame
        self._box_rel_quat = None  # (1, 4) box quat in robot root frame
        self._box_asset_name = "cardboard_box"
        # Swerve-drive L-motion (SH5): uses cmd_vel-style base motion instead of root teleport.
        self._use_swerve_l_motion = False
        self._swerve_controller = None
        self._swerve_phase = None
        self._swerve_steering_joint_ids: list[int] = []
        self._swerve_wheel_joint_ids: list[int] = []
        self._swerve_yaw_tol = 0.08
        self._swerve_max_angular_z = 0.6
        self._swerve_max_linear_x = 0.25
        # Free base driving from /cmd_vel (Plan B). Enabled per-task via
        # cfg.teleop_base_drive; replaces the scripted L-motion with live joystick control.
        self._base_drive_enabled = False
        self._cmd_vel_topic = "/cmd_vel"
        self._base_cmd_vel_timeout = 0.5
        self._latest_base_cmd_vel = (0.0, 0.0, 0.0)
        self._last_base_cmd_vel_time = 0.0
        self._cmd_vel_reader = None
        self._cmd_vel_thread = None
        # Grip detection + optional auto L-motion after sustained grasp.
        self._auto_l_on_grip_s = 0.0
        self._grip_start_time: float | None = None
        self._grip_status = "open"
        self._last_grip_status = "open"
        self._auto_l_fired_this_episode = False
        self._teleop_grip_status_path = os.environ.get(
            "EYKOREA_TELEOP_GRIP_STATUS", "/tmp/eykorea_teleop_grip.json"
        )
        self.lock = threading.Lock()  # Protect shared state

        # Initialize current joint state - will be updated only when commands are received
        self.current_joint_state = {}

        # DDS Topic Manager
        topic_manager = TopicManager(domain_id=self.domain_id)
        self._topic_manager = topic_manager  # kept so the cmd_vel reader can be added later
        trajectory_qos = self.TRAJECTORY_QOS

        # Subscribers for both arms
        self.left_arm_joint_trajectory_reader = topic_manager.topic_reader(
            topic_name="/leader/joint_trajectory_command_broadcaster_left/joint_trajectory",
            topic_type=JointTrajectory_,
            qos=trajectory_qos,
        )
        self.right_arm_joint_trajectory_reader = topic_manager.topic_reader(
            topic_name="/leader/joint_trajectory_command_broadcaster_right/joint_trajectory",
            topic_type=JointTrajectory_,
            qos=trajectory_qos,
        )
        self.head_joint_trajectory_reader = topic_manager.topic_reader(
            topic_name="/leader/joystick_controller_left/joint_trajectory",
            topic_type=JointTrajectory_,
            qos=trajectory_qos,
        )
        self.lift_joint_trajectory_reader = topic_manager.topic_reader(
            topic_name="/leader/joystick_controller_right/joint_trajectory",
            topic_type=JointTrajectory_,
            qos=trajectory_qos,
        )
        self.joystick_track_trigger_reader = topic_manager.topic_reader(
            topic_name="/leader/joystick_controller/tact_trigger",
            topic_type=String_
        )

        # Publishers
        self.joint_state_writer = topic_manager.topic_writer(
            topic_name="joint_states",
            topic_type=JointState_
        )
        self.head_cam_writer = topic_manager.topic_writer(
            topic_name="/zed/zed_node/left/image_rect_color/compressed",
            topic_type=CompressedImage_
        )
        self.right_wrist_cam_writer = topic_manager.topic_writer(
            topic_name="/camera_right/camera_right/color/image_rect_raw/compressed",
            topic_type=CompressedImage_
        )
        self.left_wrist_cam_writer = topic_manager.topic_writer(
            topic_name="/camera_left/camera_left/color/image_rect_raw/compressed",
            topic_type=CompressedImage_
        )

        # Start subscriber threads for both arms
        self.left_thread = threading.Thread(target=self._left_arm_subscriber_loop, daemon=True)
        self.right_thread = threading.Thread(target=self._right_arm_subscriber_loop, daemon=True)
        self.lift_thread = threading.Thread(target=self._lift_joint_subscriber_loop, daemon=True)
        self.head_thread = threading.Thread(target=self._head_joint_subscriber_loop, daemon=True)
        self.joystick_thread = threading.Thread(target=self._joystick_subscriber_loop, daemon=True)

        self.left_thread.start()
        self.right_thread.start()
        self.lift_thread.start()
        self.head_thread.start()
        self.joystick_thread.start()

        # Keyboard listener
        self.listener = Listener(on_press=self._on_press)
        self.listener.start()

        self._apply_env_teleop_config()
        self.joint_names = self._resolve_action_joint_order()
        if self._use_swerve_l_motion or self._base_drive_enabled:
            self._init_swerve_drive()
        if self._base_drive_enabled:
            self._start_cmd_vel_subscriber()
        self._keyboard_controls()

    def _apply_env_teleop_config(self) -> None:
        """Read optional per-task L-motion settings from ``env.cfg``."""
        cfg = getattr(self.env, "cfg", None)
        if cfg is None:
            return
        self._face_left_yaw = float(getattr(cfg, "teleop_l_yaw", self._face_left_yaw))
        self._forward_distance_m = float(getattr(cfg, "teleop_l_forward_m", self._forward_distance_m))
        self._forward_duration_s = float(
            getattr(cfg, "teleop_l_forward_duration_s", self._forward_duration_s)
        )
        self._rotation_duration_s = float(
            getattr(cfg, "teleop_l_rotation_duration_s", self._rotation_duration_s)
        )
        self._l_motion_label = str(getattr(cfg, "teleop_l_target_label", self._l_motion_label))
        self._use_swerve_l_motion = bool(getattr(cfg, "teleop_l_use_swerve", self._use_swerve_l_motion))
        self._base_drive_enabled = bool(getattr(cfg, "teleop_base_drive", self._base_drive_enabled))
        self._cmd_vel_topic = str(getattr(cfg, "teleop_cmd_vel_topic", self._cmd_vel_topic))
        self._auto_l_on_grip_s = float(
            getattr(cfg, "teleop_auto_l_on_grip_s", self._auto_l_on_grip_s)
        )

    def _resolve_action_joint_order(self) -> list[str]:
        """Match the flat action vector layout the env ActionManager applies."""
        try:
            am = self.env.action_manager
            order: list[str] = []
            for term_name in am.active_terms:
                term = am.get_term(term_name)
                joint_names = getattr(term, "_joint_names", None)
                if not joint_names:
                    raise RuntimeError(f"action term '{term_name}' exposes no joint names")
                order.extend(joint_names)
            if not order:
                raise RuntimeError("action manager produced empty joint order")
            print(f"[SG2] Action joint order resolved ({len(order)} joints).")
            return order
        except Exception as exc:
            print(f"[SG2] WARNING: using static action joint order ({exc}).")
            return list(SG2_POLICY_JOINT_NAMES)

    def _smooth_left_gripper(self, target: float | None) -> float | None:
        """Ease left gripper motion so the fingers do not snap shut."""
        if target is None:
            self._left_gripper_smooth = None
            return None
        if self._left_gripper_smooth is None:
            self._left_gripper_smooth = target
        else:
            self._left_gripper_smooth += _LEFT_GRIP_SMOOTH_ALPHA * (
                target - self._left_gripper_smooth
            )
        return self._left_gripper_smooth

    # ----------------------
    # Keyboard controls
    # ----------------------
    def _keyboard_controls(self):
        print("\n[Control] Press keys to control the FFW-SG2 robot:")
        l_motion = (
            "swerve-drive rotate + forward"
            if self._use_swerve_l_motion and self._swerve_controller is not None
            else f"smooth rotate + forward toward the {self._l_motion_label}"
        )
        if self.mode == 'record':
            print("[N / Right Joystick Button] Save successful episode and proceed to the next one")
            print("[R / Left Joystick Button] Skip failed episode (not saved) and proceed to the next one")
            print("[B / Right Joystick Button] Start recording the current episode")
            print(f"[L] {l_motion}")
            if self._auto_l_on_grip_s > 0:
                print(
                    f"[Grip] Hold the box gripped for {self._auto_l_on_grip_s:.1f}s "
                    "to auto-start L-motion (once per episode)"
                )
        elif self.mode == 'inference':
            print("[R] Skip failed episode (not saved) and proceed to the next one")
            print("[B] Start robot control")
            print(f"[L] {l_motion}")

    def _on_press(self, key):
        try:
            if hasattr(key, "char") and key.char == "l":
                with self.lock:
                    self._pending_face_left = True
                return
            if self.mode == 'record':
                if key.char == 'b':
                    self._started = True
                    self._reset_state = False
                    # Update episode tracking when manually starting
                    if self._first_episode:
                        self._first_episode = False
                    self._episode_phase = "recording"  # Now recording
                    self._auto_l_fired_this_episode = False
                elif key.char == 'r':
                    self._started = False
                    self._reset_state = True
                    self._request_pose_reset()
                    self._call_callback("R")
                    # If resetting while recording before first episode was saved, go back to first episode state
                    if self._episode_phase == "recording" and not self._first_episode:
                        self._first_episode = True
                        self._episode_phase = "idle"
                    self._reset_grip_tracking()
                elif key.char == 'n':
                    self._started = False
                    self._reset_state = True
                    self._call_callback("N")
                    # After saving, go back to idle state
                    self._episode_phase = "idle"
                    self._reset_grip_tracking()
            elif self.mode == 'inference':
                if key.char == 'b':
                    self._started = True
                    self._reset_state = False
                elif key.char == 'r':
                    self._started = False
                    self._reset_state = True
                    self._request_pose_reset()
                    self._call_callback("R")
        except AttributeError:
            pass

    def _call_callback(self, key):
        if key in self._additional_callbacks:
            self._additional_callbacks[key]()

    def _request_pose_reset(self):
        """Flag a robot root-pose reset and cancel any in-progress L motion.

        Sets booleans only (no sim access); the actual write to sim happens on
        the sim thread in ``publish_observations``. Safe to call whether or not
        ``self.lock`` is already held.
        """
        self._pending_reset_pose = True
        self._rotation_active = False
        self._forward_active = False
        self._carry_box = False

    def _advance_l_motion(self) -> None:
        """Step rotate/forward L-motion once (kinematic root + box carry)."""
        if self._use_swerve_l_motion and self._swerve_controller is not None:
            self._step_swerve_l_motion()
        else:
            self._step_yaw_rotation()
            self._step_forward_motion()

    # ----------------------
    # Subscriber loops for both arms
    # ----------------------
    def _ingest_arm_trajectory_cmd(
        self, joint_dict: dict[str, float], side: str
    ) -> dict[str, float]:
        """Split gripper commands from arm IK so they are never overwritten."""
        gripper_key = f"gripper_{side}_joint1"
        if gripper_key in joint_dict:
            gripper_val = float(joint_dict[gripper_key])
            if side == "l":
                self._left_gripper_cmd = gripper_val
            else:
                self._right_gripper_cmd = gripper_val
        return {k: v for k, v in joint_dict.items() if k != gripper_key}

    def _left_arm_subscriber_loop(self):
        """Continuously read joint trajectory commands from the leader."""
        try:
            while self.running:
                for msg in self.left_arm_joint_trajectory_reader.take_iter():
                    if msg and msg.points:
                        joint_dict = dict(zip(msg.joint_names, msg.points[-1].positions))
                        with self.lock:
                            # Merge instead of replace: arm IK and gripper commands
                            # share this topic but arrive as separate partial
                            # messages, so replacing would erase the gripper.
                            arm_only = self._ingest_arm_trajectory_cmd(joint_dict, "l")
                            self.left_arm_trajectory_cmd = self.left_arm_trajectory_cmd or {}
                            self.left_arm_trajectory_cmd.update(arm_only)
                time.sleep(0.001)  # 1ms sleep to reduce CPU load
        except Exception as e:
            print("Left arm subscriber thread exception:", e)
        finally:
            try:
                self.left_arm_joint_trajectory_reader.Close()
            except Exception as e:
                print(f"Error closing left arm subscriber: {e}")
            print("Left arm subscriber closed")

    def _right_arm_subscriber_loop(self):
        """Continuously read right arm joint trajectory commands from the leader."""
        try:
            while self.running:
                for msg in self.right_arm_joint_trajectory_reader.take_iter():
                    if msg and msg.points:
                        joint_dict = dict(zip(msg.joint_names, msg.points[-1].positions))
                        with self.lock:
                            # Merge instead of replace: arm IK and gripper commands
                            # share this topic but arrive as separate partial
                            # messages, so replacing would erase the gripper.
                            arm_only = self._ingest_arm_trajectory_cmd(joint_dict, "r")
                            self.right_arm_trajectory_cmd = self.right_arm_trajectory_cmd or {}
                            self.right_arm_trajectory_cmd.update(arm_only)
                time.sleep(0.001)  # 1ms sleep to reduce CPU load
        except Exception as e:
            print("Right arm subscriber thread exception:", e)
        finally:
            try:
                self.right_arm_joint_trajectory_reader.Close()
            except:
                pass
            print("Right arm subscriber closed")

    def _lift_joint_subscriber_loop(self):
        """Continuously read lift joint trajectory commands from the leader."""
        try:
            while self.running:
                for msg in self.lift_joint_trajectory_reader.take_iter():
                    if msg and msg.points:
                        joint_dict = dict(zip(msg.joint_names, msg.points[-1].positions))
                        with self.lock:
                            # Update only the lift joint command
                            self.lift_joint_trajectory_cmd = self.lift_joint_trajectory_cmd or {}
                            self.lift_joint_trajectory_cmd.update(joint_dict)
                time.sleep(0.001)  # 1ms sleep to reduce CPU load
        except Exception as e:
            print("Lift joint subscriber thread exception:", e)
        finally:
            try:
                self.lift_joint_trajectory_reader.Close()
            except:
                pass
            print("Lift joint subscriber closed")

    def _head_joint_subscriber_loop(self):
        """Continuously read head joint trajectory commands from the leader."""
        try:
            while self.running:
                for msg in self.head_joint_trajectory_reader.take_iter():
                    if msg and msg.points:
                        joint_dict = dict(zip(msg.joint_names, msg.points[-1].positions))
                        with self.lock:
                            # Update only the head joint commands
                            self.head_joint_trajectory_cmd = self.head_joint_trajectory_cmd or {}
                            self.head_joint_trajectory_cmd.update(joint_dict)
                time.sleep(0.001)  # 1ms sleep to reduce CPU load
        except Exception as e:
            print("Head joint subscriber thread exception:", e)
        finally:
            try:
                self.head_joint_trajectory_reader.Close()
            except:
                pass
            print("Head joint subscriber closed")

    def _joystick_subscriber_loop(self):
        """Continuously read joystick track trigger commands from the leader."""
        try:
            while self.running:
                for msg in self.joystick_track_trigger_reader.take_iter():
                    # Only process joystick triggers in record mode
                    if self.mode != 'record':
                        continue

                    joystick_trigger = msg.data
                    if joystick_trigger == 'right':
                        with self.lock:
                            if self._first_episode:
                                # First episode: only start recording
                                self._started = True
                                self._reset_state = False
                                self._first_episode = False
                                self._episode_phase = "recording"  # Now recording
                                self._auto_l_fired_this_episode = False
                            elif self._episode_phase == "recording":
                                # Currently recording: save episode and go back to idle
                                self._started = False
                                self._reset_state = True
                                self._call_callback("N")
                                self._episode_phase = "idle"  # Now idle, waiting for next start
                                self._reset_grip_tracking()
                            elif self._episode_phase == "idle":
                                # Currently idle: start new episode
                                self._started = True
                                self._reset_state = False
                                self._episode_phase = "recording"  # Now recording
                                self._auto_l_fired_this_episode = False
                    elif joystick_trigger == 'left':
                        with self.lock:
                            # Reset current episode (don't save)
                            self._started = False
                            self._reset_state = True
                            self._request_pose_reset()
                            self._call_callback("R")
                            self._reset_grip_tracking()
                            # If resetting while recording before first episode was saved, go back to first episode state
                            if self._episode_phase == "recording" and not self._first_episode:
                                # We started recording but haven't saved yet - reset to first episode
                                self._first_episode = True
                                self._episode_phase = "idle"
                time.sleep(0.001)  # 1ms sleep to reduce CPU load
        except Exception as e:
            print("Joystick subscriber thread exception:", e)
        finally:
            try:
                self.joystick_track_trigger_reader.Close()
            except:
                pass
            print("Joystick subscriber closed")

    # ----------------------
    # Publishers
    # ----------------------
    def _publish_joint_states(self):
        """Publish current joint states over DDS."""
        now = datetime.now()
        stamp = Time_(sec=int(now.timestamp()), nanosec=now.microsecond * 1000)
        header = Header_(stamp=stamp, frame_id="base_link")

        obs_joint_name = self.env.scene["robot"].data.joint_names
        all_positions = self.env.scene["robot"].data.joint_pos.squeeze(0).tolist()
        all_velocities = self.env.scene["robot"].data.joint_vel.squeeze(0).tolist()
        all_efforts = [0.0] * len(all_positions)

        # Flatten nested lists if necessary
        if isinstance(all_positions[0], list):
            all_positions = [p for sub in all_positions for p in sub]
        if isinstance(all_velocities[0], list):
            all_velocities = [v for sub in all_velocities for v in sub]

        # Get indices of the joints we care about
        indices = [obs_joint_name.index(name) for name in self.joint_names if name in obs_joint_name]

        positions = [all_positions[i] for i in indices]
        velocities = [all_velocities[i] for i in indices]
        efforts = [all_efforts[i] for i in indices]

        joint_state = JointState_(
            header=header,
            name=[self.joint_names[i] for i in range(len(indices))],
            position=positions,
            velocity=velocities,
            effort=efforts
        )

        try:
            self.joint_state_writer.write(joint_state)
        except Exception as e:
            print("[Writer] write error:", e)

    def _publish_camera(self, cam_name: str):
        """Publish camera image as DDS compressed image."""
        try:
            cam_data = self.env.scene[cam_name].data
            img = cam_data.output['rgb'][0].cpu().numpy()  # Convert tensor to numpy (RGB format)
            
            # Convert RGB to BGR for OpenCV encoding
            img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

            _, buffer = cv2.imencode('.jpg', img_bgr)
            jpeg_bytes = buffer.tobytes()

            now = datetime.now()
            stamp = Time_(sec=int(now.timestamp()), nanosec=now.microsecond * 1000)
            header = Header_(stamp=stamp, frame_id="camera_frame")

            msg = CompressedImage_(header=header, format="jpeg", data=jpeg_bytes)
            
            # Map camera names to publishers for FFW_SG2
            if cam_name == "cam_wrist_right":
                self.right_wrist_cam_writer.write(msg)
            elif cam_name == "cam_wrist_left":
                self.left_wrist_cam_writer.write(msg)
            elif cam_name == "cam_head":
                self.head_cam_writer.write(msg)
        except Exception as e:
            print(f"Camera publish error for {cam_name}:", e)

    def _compute_action_state(self):
        """Compute current action dictionary based on keyboard input and subscriber."""
        state = {'reset': self._reset_state, 'started': self._started}
        if state['reset']:
            self._reset_state = False
            return state
        state['joint_state'] = self._get_device_state()
        return state

    def _get_device_state(self):
        """Return latest joint positions for the action vector."""
        return self._build_live_joint_state()

    def _build_live_joint_state(self) -> dict[str, float]:
        """Merge sim state with the latest DDS teleop commands."""
        with self.lock:
            # Start with current robot joint positions
            obs_joint_name = self.env.scene["robot"].data.joint_names
            all_positions = self.env.scene["robot"].data.joint_pos.squeeze(0).tolist()
            
            # Flatten nested lists if necessary
            if isinstance(all_positions[0], list):
                all_positions = [p for sub in all_positions for p in sub]

            # Build joint state from current robot state
            joint_state = {}
            for name in self.joint_names:
                if name in obs_joint_name:
                    idx = obs_joint_name.index(name)
                    joint_state[name] = all_positions[idx]
                else:
                    joint_state[name] = 0.0  # Fallback only if joint not found in robot
            
            # Update with left arm commands if available
            if self.left_arm_trajectory_cmd:
                joint_state.update(self.left_arm_trajectory_cmd)
            
            # Update with right arm commands if available
            if self.right_arm_trajectory_cmd:
                joint_state.update(self.right_arm_trajectory_cmd)

            if self._left_gripper_cmd is not None:
                grip = self._smooth_left_gripper(self._left_gripper_cmd)
                if grip is not None and "gripper_l_joint1" in self.joint_names:
                    joint_state["gripper_l_joint1"] = grip
            if self._right_gripper_cmd is not None:
                joint_state["gripper_r_joint1"] = self._right_gripper_cmd

            if self.head_joint_trajectory_cmd:
                joint_state.update(self.head_joint_trajectory_cmd)

            if self.lift_joint_trajectory_cmd:
                joint_state.update(self.lift_joint_trajectory_cmd)
            
            return joint_state

    def get_action(self):
        """Return action tensor for robot control."""
        action = self._compute_action_state()
        if action['reset']:
            return {"reset": True}
        if not action['started']:
            return None

        joint_state = action['joint_state']
        positions = [joint_state.get(name, 0.0) for name in self.joint_names]
        return torch.tensor(
            positions, device=self.env.device, dtype=torch.float32
        ).unsqueeze(0)

    def after_env_step(self) -> None:
        """Post-physics hooks: finger stiffening and box carry sync."""
        if self._carry_box:
            self._apply_box_carry()
        self._blend_left_gripper_fingers()

    def _left_gripper_hold_value(self) -> float | None:
        with self.lock:
            raw = self._left_gripper_cmd
        if raw is None:
            return self._left_gripper_smooth
        return self._smooth_left_gripper(raw)

    def _blend_left_gripper_fingers(self) -> None:
        """Blend mimic finger joints toward the smoothed close angle (not joint1)."""
        grip = self._left_gripper_hold_value()
        if grip is None or grip < _LEFT_FINGER_HOLD_MIN:
            return

        robot = self.env.scene["robot"]
        try:
            finger_ids = [robot.joint_names.index(f"gripper_l_joint{i}") for i in range(2, 5)]
        except ValueError:
            return

        hold_alpha = (
            _LEFT_FINGER_HOLD_ALPHA_FIRM if grip >= 0.45 else _LEFT_FINGER_HOLD_ALPHA
        )
        device = self.env.device
        env_ids = torch.tensor([0], device=device)
        grip_t = float(grip)

        joint_pos = robot.data.joint_pos.clone()
        joint_vel = robot.data.joint_vel.clone()
        for finger_id in finger_ids:
            current = float(joint_pos[0, finger_id].item())
            blended = current + hold_alpha * (grip_t - current)
            joint_pos[0, finger_id] = blended
            joint_vel[0, finger_id] = 0.0
        robot.write_joint_state_to_sim(joint_pos, joint_vel, env_ids=env_ids)

        pos_target = robot.data.joint_pos_target.clone()
        vel_target = robot.data.joint_vel_target.clone()
        for finger_id in finger_ids:
            pos_target[:, finger_id] = joint_pos[0, finger_id]
            vel_target[:, finger_id] = 0.0
        robot.set_joint_position_target(pos_target)
        robot.set_joint_velocity_target(vel_target)

    def publish_observations(self):
        """Publish joint states and camera images."""
        if not self._base_drive_enabled:
            self._step_grip_auto_l()
        # Capture the robot's home root pose once, before any L motion edits it.
        if self._home_root_pose is None:
            robot = self.env.scene["robot"]
            self._home_root_pose = robot.data.root_state_w[0:1, 0:7].clone()

        with self.lock:
            pending_face_left = self._pending_face_left
            self._pending_face_left = False
            pending_reset_pose = self._pending_reset_pose
            self._pending_reset_pose = False
        # Pose reset (key 'R') takes priority over a queued face-left request.
        if pending_reset_pose:
            self._restore_home_pose()
        elif pending_face_left and not self._base_drive_enabled:
            self.face_left_table()

        if self._base_drive_enabled:
            # Free driving: cmd_vel steers the wheels physically, no scripted L-motion.
            self._apply_base_cmd_vel()
        elif self._is_l_motion_active():
            self._advance_l_motion()

        self._publish_joint_states()
        self._publish_camera("cam_head")
        # self._publish_camera("cam_wrist_right")
        # self._publish_camera("cam_wrist_left")

    # ----------------------
    # Utility
    # ----------------------
    def shutdown(self):
        """Stop threads and close DDS publishers/subscribers."""
        self.running = False
        self.left_thread.join()
        self.right_thread.join()
        self.lift_thread.join()
        self.head_thread.join()
        self.joystick_thread.join()
        
        for obj in [self.left_arm_joint_trajectory_reader, self.right_arm_joint_trajectory_reader,
                    self.joint_state_writer, self.head_cam_writer, 
                    self.right_wrist_cam_writer, self.left_wrist_cam_writer]:
            try:
                obj.Close()
            except:
                pass
        print("FFWSG2Sdk shutdown complete")

    def reset(self):
        self._reset_state = False
        self._rotation_active = False
        self._forward_active = False
        self._pending_reset_pose = False
        self._pending_face_left = False
        self._carry_box = False
        self._swerve_phase = None
        self._reset_grip_tracking()

    def on_episode_saved(self) -> None:
        """Match VR/joystick state after auto-save (same as manual N / right trigger)."""
        with self.lock:
            self._started = False
            self._reset_state = True
            self._episode_phase = "idle"
        self._rotation_active = False
        self._forward_active = False
        self._pending_reset_pose = False
        self._pending_face_left = False
        self._carry_box = False
        self._swerve_phase = None
        self._reset_grip_tracking()

    def _reset_grip_tracking(self) -> None:
        self._grip_start_time = None
        self._auto_l_fired_this_episode = False
        self._grip_status = "open"
        self._last_grip_status = "open"
        self._left_gripper_smooth = None
        self._write_grip_status("open", 0.0)

    def is_l_motion_active(self) -> bool:
        """True while rotate/forward L-motion is running."""
        return self._is_l_motion_active()

    def use_kinematic_l_record_step(self) -> bool:
        """During demo recording, L-motion should skip physics integration."""
        return self._started and self._is_l_motion_active()

    def sync_kinematic_state(self) -> None:
        """After kinematic L-motion, keep the carried box aligned with the base."""
        if self._carry_box:
            self._apply_box_carry()

    def _is_l_motion_active(self) -> bool:
        if self._swerve_phase is not None:
            return True
        return self._rotation_active or self._forward_active

    def _motion_alpha(self, *, start_time: float, duration_s: float, sim_alpha: float) -> float:
        """Smoothstep progress: fixed sim dt per recorded frame, wall clock when idle."""
        if self._started:
            return self._smoothstep(min(sim_alpha, 1.0))
        elapsed = time.monotonic() - start_time
        return self._smoothstep(min(elapsed / duration_s, 1.0))

    def _check_box_gripped(self) -> bool:
        """True when both end effectors grasp the task box (same obs as env cfg)."""
        try:
            terms = self.env.observation_manager.compute_group("subtask_terms")
            if "dual_grasp_box" in terms:
                return bool(terms["dual_grasp_box"][0].item())
        except Exception:
            pass
        return False

    def _write_grip_status(self, state: str, held_s: float) -> None:
        auto_l_in = None
        if state == "gripped" and self._auto_l_on_grip_s > 0:
            auto_l_in = max(0.0, self._auto_l_on_grip_s - held_s)
        payload = {
            "state": state,
            "held_s": round(held_s, 1),
            "auto_l_in_s": round(auto_l_in, 1) if auto_l_in is not None else None,
            "auto_l_enabled": self._auto_l_on_grip_s > 0,
        }
        try:
            with open(self._teleop_grip_status_path, "w", encoding="utf-8") as f:
                json.dump(payload, f)
        except OSError:
            pass

    def _step_grip_auto_l(self) -> None:
        gripped = self._check_box_gripped()
        now = time.monotonic()

        if gripped:
            if self._grip_start_time is None:
                self._grip_start_time = now
                if self._last_grip_status != "gripped":
                    if self._auto_l_on_grip_s > 0:
                        print(
                            f"[Teleop] Box gripped — hold {self._auto_l_on_grip_s:.1f}s "
                            "to auto-start L-motion"
                        )
                    else:
                        print("[Teleop] Box gripped")
            held_s = now - self._grip_start_time
            self._grip_status = "gripped"
            self._write_grip_status("gripped", held_s)

            if (
                self._auto_l_on_grip_s > 0
                and self.mode == "record"
                and self._episode_phase == "recording"
                and self._started
                and not self._auto_l_fired_this_episode
                and not self._is_l_motion_active()
                and held_s >= self._auto_l_on_grip_s
            ):
                with self.lock:
                    self._pending_face_left = True
                self._auto_l_fired_this_episode = True
                print(
                    f"[Teleop] Box held {self._auto_l_on_grip_s:.1f}s — "
                    "auto-starting L-motion"
                )
        else:
            if self._last_grip_status == "gripped":
                print("[Teleop] Grip released")
            self._grip_start_time = None
            self._grip_status = "open"
            self._write_grip_status("open", 0.0)

        self._last_grip_status = self._grip_status

    def _restore_home_pose(self):
        """Restore the robot root pose (position + orientation) to its home/start
        pose, undoing any rotation/translation applied by the L (face-left) action."""
        self._swerve_phase = None
        if self._swerve_controller is not None:
            self._apply_swerve_cmd_vel(0.0, 0.0, 0.0)
        if self._home_root_pose is None:
            return
        robot = self.env.scene["robot"]
        device = self.env.device
        env_ids = torch.tensor([0], device=device)
        root_pose = self._home_root_pose.to(device)
        robot.write_root_pose_to_sim(root_pose, env_ids=env_ids)
        robot.write_root_velocity_to_sim(torch.zeros(1, 6, device=device), env_ids=env_ids)
        print("[Control] Robot root pose reset to home")

    def face_left_table(self):
        """Begin a smooth rotation toward the task target, then move forward."""
        self._auto_l_fired_this_episode = True
        self._capture_box_carry()
        if self._use_swerve_l_motion and self._swerve_controller is not None:
            self._forward_active = False
            self._rotation_active = False
            self._swerve_phase = "rotate"
            self._begin_yaw_rotation(self._face_left_yaw)
            print(
                f"[Control] Swerve: rotating to face the {self._l_motion_label}, "
                f"then driving forward {self._forward_distance_m:.2f} m"
            )
            return
        self._forward_active = False
        self._begin_yaw_rotation(self._face_left_yaw)
        print(f"[Control] Rotating robot to face the {self._l_motion_label}")

    def _capture_box_carry(self):
        """Record box pose in robot root frame for rigid carry during L-motion."""
        try:
            robot = self.env.scene["robot"]
            box = self.env.scene[self._box_asset_name]
        except KeyError:
            self._carry_box = False
            return

        root_pos = robot.data.root_pos_w[0:1].clone()
        root_quat = robot.data.root_quat_w[0:1].clone()
        box_pos = box.data.root_pos_w[0:1].clone()
        box_quat = box.data.root_quat_w[0:1].clone()

        inv_root_quat = math_utils.quat_inv(root_quat)
        self._box_rel_pos = math_utils.quat_apply(inv_root_quat, box_pos - root_pos)
        self._box_rel_quat = math_utils.quat_mul(inv_root_quat, box_quat)
        self._carry_box = True

    def _apply_box_carry(self):
        """Re-impose the captured box->root relative transform."""
        if not self._carry_box or self._box_rel_pos is None:
            return
        try:
            robot = self.env.scene["robot"]
            box = self.env.scene[self._box_asset_name]
        except KeyError:
            return

        device = self.env.device
        env_ids = torch.tensor([0], device=device)
        root_pos = robot.data.root_pos_w[0:1]
        root_quat = robot.data.root_quat_w[0:1]
        rel_pos = self._box_rel_pos.to(device)
        rel_quat = self._box_rel_quat.to(device)
        box_pos = root_pos + math_utils.quat_apply(root_quat, rel_pos)
        box_quat = math_utils.quat_mul(root_quat, rel_quat)
        box_pose = torch.cat([box_pos, box_quat], dim=-1)
        box.write_root_pose_to_sim(box_pose, env_ids=env_ids)
        box.write_root_velocity_to_sim(torch.zeros(1, 6, device=device), env_ids=env_ids)

    def _get_robot_yaw(self) -> float:
        robot = self.env.scene["robot"]
        _, _, yaw = math_utils.euler_xyz_from_quat(robot.data.root_quat_w[0:1])
        return float(yaw.item())

    def _begin_yaw_rotation(self, target_yaw: float):
        self._rotation_start_yaw = self._get_robot_yaw()
        self._rotation_target_yaw = target_yaw
        self._rotation_start_time = time.monotonic()
        self._rotation_sim_alpha = 0.0
        self._rotation_active = True

    def _begin_forward_motion(self):
        robot = self.env.scene["robot"]
        device = self.env.device
        self._forward_start_pos = robot.data.root_pos_w[0:1].clone()
        quat = robot.data.root_quat_w[0:1].clone()
        offset = torch.tensor(
            [[self._forward_distance_m, 0.0, 0.0]],
            device=device,
        )
        self._forward_end_pos = self._forward_start_pos + math_utils.quat_apply(quat, offset)
        self._forward_yaw = self._get_robot_yaw()
        self._forward_start_time = time.monotonic()
        self._forward_sim_alpha = 0.0
        self._forward_active = True

    def _smoothstep(self, alpha: float) -> float:
        return alpha * alpha * (3.0 - 2.0 * alpha)

    def _lerp_yaw(self, start: float, end: float, alpha: float) -> float:
        delta = (end - start + math.pi) % (2.0 * math.pi) - math.pi
        return start + alpha * delta

    def _set_robot_pose(self, pos: torch.Tensor, yaw: float):
        robot = self.env.scene["robot"]
        device = self.env.device
        env_ids = torch.tensor([0], device=device)
        quat = math_utils.quat_from_euler_xyz(
            torch.zeros(1, device=device),
            torch.zeros(1, device=device),
            torch.tensor([yaw], device=device),
        )
        root_pose = torch.cat([pos, quat], dim=-1)
        robot.write_root_pose_to_sim(root_pose, env_ids=env_ids)
        robot.write_root_velocity_to_sim(torch.zeros(1, 6, device=device), env_ids=env_ids)

    def _set_robot_yaw(self, yaw: float):
        robot = self.env.scene["robot"]
        self._set_robot_pose(robot.data.root_pos_w[0:1].clone(), yaw)

    def _step_yaw_rotation(self):
        if not self._rotation_active:
            return

        if self._started:
            self._rotation_sim_alpha = min(
                1.0,
                self._rotation_sim_alpha + self._sim_step_dt() / self._rotation_duration_s,
            )
            alpha = self._motion_alpha(
                start_time=self._rotation_start_time,
                duration_s=self._rotation_duration_s,
                sim_alpha=self._rotation_sim_alpha,
            )
        else:
            alpha = self._motion_alpha(
                start_time=self._rotation_start_time,
                duration_s=self._rotation_duration_s,
                sim_alpha=0.0,
            )
        yaw = self._lerp_yaw(self._rotation_start_yaw, self._rotation_target_yaw, alpha)
        self._set_robot_yaw(yaw)
        self._apply_box_carry()

        if alpha >= 1.0:
            self._rotation_active = False
            self._begin_forward_motion()
            print(f"[Control] Robot finished rotating; moving forward toward the {self._l_motion_label}")

    def _step_forward_motion(self):
        if not self._forward_active:
            return

        if self._started:
            self._forward_sim_alpha = min(
                1.0,
                self._forward_sim_alpha + self._sim_step_dt() / self._forward_duration_s,
            )
            alpha = self._motion_alpha(
                start_time=self._forward_start_time,
                duration_s=self._forward_duration_s,
                sim_alpha=self._forward_sim_alpha,
            )
        else:
            alpha = self._motion_alpha(
                start_time=self._forward_start_time,
                duration_s=self._forward_duration_s,
                sim_alpha=0.0,
            )
        pos = self._forward_start_pos + alpha * (self._forward_end_pos - self._forward_start_pos)
        self._set_robot_pose(pos, self._forward_yaw)
        self._apply_box_carry()

        if alpha >= 1.0:
            self._forward_active = False
            self._carry_box = False
            print(f"[Control] Robot finished moving forward toward the {self._l_motion_label}")

    def _resolve_swerve_profile(self, joint_names: list[str]):
        """Return swerve constants for the loaded robot articulation."""
        names = set(joint_names)
        from cyclo_lab.assets.robots.FFW_SG2 import (
            SG2_SWERVE_MODULE_ANGLE_OFFSETS,
            SG2_SWERVE_MODULE_X_OFFSETS,
            SG2_SWERVE_MODULE_Y_OFFSETS,
            SG2_SWERVE_STEERING_JOINTS,
            SG2_SWERVE_WHEEL_JOINTS,
            SG2_SWERVE_WHEEL_RADIUS,
        )
        from cyclo_lab.assets.robots.FFW_SH5 import (
            SH5_SWERVE_MODULE_ANGLE_OFFSETS,
            SH5_SWERVE_MODULE_X_OFFSETS,
            SH5_SWERVE_MODULE_Y_OFFSETS,
            SH5_SWERVE_STEERING_JOINTS,
            SH5_SWERVE_WHEEL_JOINTS,
            SH5_SWERVE_WHEEL_RADIUS,
        )

        profiles = (
            (
                "SH5",
                SH5_SWERVE_STEERING_JOINTS,
                SH5_SWERVE_WHEEL_JOINTS,
                SH5_SWERVE_MODULE_X_OFFSETS,
                SH5_SWERVE_MODULE_Y_OFFSETS,
                SH5_SWERVE_MODULE_ANGLE_OFFSETS,
                SH5_SWERVE_WHEEL_RADIUS,
            ),
            (
                "SG2",
                SG2_SWERVE_STEERING_JOINTS,
                SG2_SWERVE_WHEEL_JOINTS,
                SG2_SWERVE_MODULE_X_OFFSETS,
                SG2_SWERVE_MODULE_Y_OFFSETS,
                SG2_SWERVE_MODULE_ANGLE_OFFSETS,
                SG2_SWERVE_WHEEL_RADIUS,
            ),
        )
        for label, steering, wheels, x_off, y_off, ang_off, radius in profiles:
            if all(j in names for j in steering) and all(j in names for j in wheels):
                return label, steering, wheels, x_off, y_off, ang_off, radius
        return None

    def _init_swerve_drive(self) -> None:
        """Set up the swerve controller (same stack as sh5_dds_bringup)."""
        import sys
        from pathlib import Path

        bringup_dir = Path(__file__).resolve().parents[2] / "bringup"
        if str(bringup_dir) not in sys.path:
            sys.path.insert(0, str(bringup_dir))

        from common import robotis_config as bringup_cfg
        from common.swerve_drive import SwerveDriveController, SwerveModule

        robot = self.env.scene["robot"]
        joint_names = list(robot.data.joint_names)
        name_to_id = {name: idx for idx, name in enumerate(joint_names)}

        profile = self._resolve_swerve_profile(joint_names)
        if profile is None:
            print("[Swerve] No matching swerve joint set in articulation; L-motion uses root teleport.")
            self._use_swerve_l_motion = False
            return

        robot_label, steering_joints, wheel_joints, x_offsets, y_offsets, angle_offsets, wheel_radius = (
            profile
        )

        modules = [
            SwerveModule(
                steering_joint=steering_joint,
                wheel_joint=wheel_joint,
                x_offset=x_offsets[index],
                y_offset=y_offsets[index],
                angle_offset=angle_offsets[index],
                steering_limit_lower=bringup_cfg.AI_WORKER_SWERVE_STEERING_LIMIT_LOWER,
                steering_limit_upper=bringup_cfg.AI_WORKER_SWERVE_STEERING_LIMIT_UPPER,
                wheel_speed_limit_lower=bringup_cfg.AI_WORKER_SWERVE_WHEEL_SPEED_LIMIT_LOWER,
                wheel_speed_limit_upper=bringup_cfg.AI_WORKER_SWERVE_WHEEL_SPEED_LIMIT_UPPER,
            )
            for index, (steering_joint, wheel_joint) in enumerate(zip(steering_joints, wheel_joints))
        ]

        missing = [
            joint
            for module in modules
            for joint in (module.steering_joint, module.wheel_joint)
            if joint not in name_to_id
        ]
        if missing:
            print(f"[{robot_label}] Swerve joints missing from articulation {missing}; L-motion uses root teleport.")
            self._use_swerve_l_motion = False
            return

        self._swerve_steering_joint_ids = [name_to_id[m.steering_joint] for m in modules]
        self._swerve_wheel_joint_ids = [name_to_id[m.wheel_joint] for m in modules]
        self._swerve_controller = SwerveDriveController(modules, wheel_radius)
        self._use_swerve_l_motion = True
        print(f"[{robot_label}] Swerve-drive L-motion enabled (cmd_vel-style base control).")

    # ----------------------
    # Free base driving from /cmd_vel (Plan B)
    # ----------------------
    def _start_cmd_vel_subscriber(self) -> None:
        """Subscribe to the VR base twist and drive the swerve wheels live."""
        if self._swerve_controller is None:
            print("[BaseDrive] Swerve controller unavailable; base driving disabled.")
            self._base_drive_enabled = False
            return
        self._cmd_vel_reader = self._topic_manager.topic_reader(
            topic_name=self._cmd_vel_topic,
            topic_type=Twist_,
            qos=self.TRAJECTORY_QOS,
        )
        self._cmd_vel_thread = threading.Thread(
            target=self._cmd_vel_subscriber_loop, daemon=True
        )
        self._cmd_vel_thread.start()
        print(f"[BaseDrive] cmd_vel base driving enabled on {self._cmd_vel_topic}.")

    def _cmd_vel_subscriber_loop(self) -> None:
        try:
            while self.running:
                for msg in self._cmd_vel_reader.take_iter():
                    self._store_base_cmd_vel(msg)
                time.sleep(0.001)
        except Exception as exc:  # noqa: BLE001
            print(f"[BaseDrive] cmd_vel subscriber exception: {exc}")

    def _store_base_cmd_vel(self, msg) -> None:
        if msg is None:
            return
        with self.lock:
            self._latest_base_cmd_vel = (
                float(msg.linear.x), float(msg.linear.y), float(msg.angular.z)
            )
            self._last_base_cmd_vel_time = time.monotonic()

    def _current_base_cmd_vel(self) -> tuple[float, float, float]:
        with self.lock:
            command = self._latest_base_cmd_vel
            last = self._last_base_cmd_vel_time
        if last == 0.0:
            return 0.0, 0.0, 0.0
        if self._base_cmd_vel_timeout > 0.0 and (time.monotonic() - last) > self._base_cmd_vel_timeout:
            return 0.0, 0.0, 0.0  # stale command -> stop the base
        return command

    def _apply_base_cmd_vel(self) -> None:
        """Drive the swerve base physically from the latest cmd_vel (no root teleport)."""
        if self._swerve_controller is None:
            return
        vx, vy, angular_z = self._current_base_cmd_vel()
        self._apply_swerve_cmd_vel(vx, vy, angular_z, integrate_root=False)

    def _sim_step_dt(self) -> float:
        """Environment step duration (physics dt * decimation), not wall clock."""
        try:
            return max(float(self.env.step_dt), 1.0e-3)
        except AttributeError:
            return 1.0 / 60.0

    def _apply_swerve_cmd_vel(
        self,
        linear_x: float,
        linear_y: float,
        angular_z: float,
        *,
        integrate_root: bool = True,
        dt=None,
    ) -> None:
        if self._swerve_controller is None:
            return

        robot = self.env.scene["robot"]
        step_dt = dt if dt is not None else self._sim_step_dt()

        current_steering = [
            float(v)
            for v in robot.data.joint_pos[0, self._swerve_steering_joint_ids].detach().cpu().tolist()
        ]
        current_wheel_velocities = [
            float(v)
            for v in robot.data.joint_vel[0, self._swerve_wheel_joint_ids].detach().cpu().tolist()
        ]

        module_commands = self._swerve_controller.compute_commands(
            linear_x,
            linear_y,
            angular_z,
            current_steering_positions=current_steering,
            current_wheel_velocities=current_wheel_velocities,
            dt=step_dt,
        )

        position_target = robot.data.joint_pos_target.clone()
        velocity_target = robot.data.joint_vel_target.clone()
        for module_command, steering_id, wheel_id in zip(
            module_commands,
            self._swerve_steering_joint_ids,
            self._swerve_wheel_joint_ids,
        ):
            position_target[:, steering_id] = module_command.steering_position
            velocity_target[:, wheel_id] = module_command.wheel_velocity
        robot.set_joint_position_target(position_target)
        robot.set_joint_velocity_target(velocity_target)

        if not integrate_root:
            return

        # Fallback path when not using the smooth S-curve pose driver.
        if (
            abs(linear_x) > 1.0e-4
            or abs(linear_y) > 1.0e-4
            or abs(angular_z) > 1.0e-4
        ):
            self._integrate_swerve_root(linear_x, linear_y, angular_z, step_dt)

    def _integrate_swerve_root(
        self, linear_x: float, linear_y: float, angular_z: float, dt: float
    ) -> None:
        robot = self.env.scene["robot"]
        device = self.env.device
        env_ids = torch.tensor([0], device=device)

        yaw = self._get_robot_yaw()
        new_yaw = yaw + angular_z * dt
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        dx = (linear_x * cos_y - linear_y * sin_y) * dt
        dy = (linear_x * sin_y + linear_y * cos_y) * dt

        pos = robot.data.root_pos_w[0:1].clone()
        pos[0, 0] += dx
        pos[0, 1] += dy
        quat = math_utils.quat_from_euler_xyz(
            torch.zeros(1, device=device),
            torch.zeros(1, device=device),
            torch.tensor([new_yaw], device=device),
        )
        root_pose = torch.cat([pos, quat], dim=-1)
        robot.write_root_pose_to_sim(root_pose, env_ids=env_ids)
        robot.write_root_velocity_to_sim(torch.zeros(1, 6, device=device), env_ids=env_ids)
        self._apply_box_carry()

    def _step_swerve_l_motion(self) -> None:
        if self._swerve_phase is None:
            self._apply_swerve_cmd_vel(0.0, 0.0, 0.0, integrate_root=False)
            return

        robot = self.env.scene["robot"]
        step_dt = self._sim_step_dt()

        if self._swerve_phase == "rotate":
            elapsed = time.monotonic() - self._rotation_start_time
            alpha = self._smoothstep(min(elapsed / self._rotation_duration_s, 1.0))
            target_yaw = self._lerp_yaw(
                self._rotation_start_yaw, self._rotation_target_yaw, alpha
            )
            self._set_robot_pose(robot.data.root_pos_w[0:1].clone(), target_yaw)
            self._apply_box_carry()
            # Wheels stay still: base motion is kinematic root teleport. Spinning
            # wheels here vibrates the arms and shakes the box out of the grippers.
            self._apply_swerve_cmd_vel(
                0.0, 0.0, 0.0, integrate_root=False, dt=step_dt
            )

            if alpha >= 1.0:
                self._swerve_phase = "forward"
                self._begin_forward_motion()
                print(f"[Control] Swerve: rotation done; driving toward the {self._l_motion_label}")
            return

        if self._swerve_phase == "forward":
            elapsed = time.monotonic() - self._forward_start_time
            alpha = self._smoothstep(min(elapsed / self._forward_duration_s, 1.0))
            pos = self._forward_start_pos + alpha * (
                self._forward_end_pos - self._forward_start_pos
            )
            self._set_robot_pose(pos, self._forward_yaw)
            self._apply_box_carry()

            self._apply_swerve_cmd_vel(
                0.0, 0.0, 0.0, integrate_root=False, dt=step_dt
            )

            if alpha >= 1.0:
                self._swerve_phase = None
                self._carry_box = False
                self._apply_swerve_cmd_vel(0.0, 0.0, 0.0, integrate_root=False)
                print(f"[Control] Swerve: finished driving toward the {self._l_motion_label}")

    def add_callback(self, key: str, func: Callable):
        """Add callback function for a specific key."""
        self._additional_callbacks[key] = func
