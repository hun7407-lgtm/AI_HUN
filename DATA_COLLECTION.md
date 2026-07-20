# Data Collection Pipeline — FFW-SG2 VR Teleoperation

**Author:** Hun Kim (hun7728@hanyang.ac.kr) · **Last updated:** 2026-07-20

End-to-end guide for collecting bimanual manipulation **and mobile-base** demonstrations in
NVIDIA Isaac Sim through Meta Quest VR teleoperation, and exporting them to the **LeRobot**
format that matches the physical `ffw_sg2_rev1` robot's data schema.

This document is ordered as the data-collection workflow actually runs: **setup → environment
→ real-robot parity → teleoperation → recording → conversion**. Sections that describe work
added on top of the upstream ROBOTIS stack are marked **[ADDED]** or **[MODIFIED]**, and every
touched file is listed in [Appendix A](#appendix-a--files-added--modified).

---

## 1. Overview

| | |
|---|---|
| **Goal** | Collect demonstrations whose format matches the real robot, so sim and real data can be pooled for imitation learning. |
| **Robot** | ROBOTIS FFW-SG2 — dual 7-DoF arms + parallel grippers, head (2-DoF), vertical lift, and a 3-module swerve base. |
| **Task** | L-table pick & place (grasp a box on the front table, carry it, place it on the left table). |
| **Engine** | Isaac Sim **5.1.0** + Isaac Lab **2.3.0**. |
| **Input** | Human operator with a Meta Quest 3 (WebXR / Vuer), teleoperating the arms **and driving the base**. |
| **Output** | LeRobot dataset — 4 RGB camera streams + 22-dim state/action (19 joints + base velocity). |

### Data flow

```
[Operator + Meta Quest 3]
      │  hand/controller pose + joystick (WebXR)
      ▼
[robotis-applications] Vuer VR publisher ──/cmd_vel + arm poses (ROS 2 / DDS, domain 30)──┐
      │                                                                                    │
      ▼                                                                                    ▼
[ai_worker] vr_controller (arm IK)                                        [cyclo_lab] FFWSG2Sdk
      │  joint commands (DDS)                                                   swerve base driving
      ▼                                                                                    │
[cyclo_lab] Isaac Sim + record_demos.py ◄──────────────────────────────────────────────────┘
      │  observations (joints + base velocity + 4 cameras)
      ▼
   raw .hdf5  ──►  IK convert ──►  annotate ──►  (Mimic datagen)  ──►  joint convert  ──►  LeRobot
```

---

## 2. System Setup

### 2.1 Prerequisites (not provided by `setup.sh`)

| Requirement | Check |
|---|---|
| Linux host (Ubuntu / Pop!_OS) + X11 | — |
| NVIDIA RTX GPU + driver (validated: RTX PRO 5000, 24 GB) | `nvidia-smi` |
| Docker Engine + NVIDIA Container Toolkit | `docker run --gpus all …` |
| NGC login (pulls `nvcr.io/nvidia/isaac-sim:5.1.0`) | `docker login nvcr.io` |
| Meta Quest 3 (+ one-time ADB/udev for USB tether) | `adb_vr_connect/README.md` |

### 2.2 Clone and build

```bash
git clone https://github.com/hun7407-lgtm/AI_HUN.git AIWORKER
cd AIWORKER
./setup.sh ~/AIWORKER        # clones the 3 upstream repos at pinned commits + applies overlays
```

`setup.sh` produces `~/AIWORKER/{cyclo_lab, ai_worker, robotis_applications}`. This repo is a
thin **overlay** — it version-controls only the changed files under `overlays/`; the upstream
sources and recorded datasets are excluded.

### 2.3 Start the three containers

```bash
cd ~/AIWORKER/cyclo_lab/docker            && ./container.sh start   # Isaac Sim, recording, dashboard :8765
cd ~/AIWORKER/robotis_applications/docker && ./container.sh start   # Vuer VR publisher :8012
cd ~/AIWORKER/ai_worker/docker            && ./container.sh start   # arm IK controller
```

All three share `ROS_DOMAIN_ID=30` (DDS / `rmw_fastrtps_cpp`).

---

## 3. Simulation Environment

Two variants of the L-table task exist. **The stock task is unchanged**; the mobile task is
**[ADDED]** for base-velocity data collection.

| Task ID | Base | Base motion | State/action | Use |
|---|---|---|---|---|
| `Cyclo-Real-Pick-Place-LTable-FFW-SG2-v0` | Welded (fixed) | Scripted L-motion (kinematic teleport) | 19-dim | Stock (arms only) |
| `Cyclo-Real-Pick-Place-LTable-Mobile-FFW-SG2-v0` **[ADDED]** | Free (drivable) | Operator drives via `/cmd_vel` (physical swerve) | 22-dim | Mobile-base collection |

---

## 4. Real-Robot Parity: Cameras **[MODIFIED]**

The physical `ffw_sg2_rev1` records **four** RGB streams; the stock sim rendered only one head
camera. Three cameras were added so recordings match the real observation schema.

| LeRobot key | Sim camera | Mount | Resolution |
|---|---|---|---|
| `observation.images.rgb.cam_left_head` | `cam_head` (ZED left eye) | `head_link2/zed` | 376 × 672 |
| `observation.images.rgb.cam_right_head` | `cam_right_head` | `head_link2/zed` (mirrored) | 376 × 672 |
| `observation.images.rgb.cam_left_wrist` | `cam_left_wrist` | `arm_l_link7` (D405 pose) | 424 × 240 |
| `observation.images.rgb.cam_right_wrist` | `cam_right_wrist` | `arm_r_link7` (D405 pose) | 424 × 240 |

Key points:
- **Head cameras** are the two eyes of the head ZED, placed symmetrically about the `zed`
  prim centre (±0.03 m in Y ⇒ ~0.06 m baseline).
- **Wrist cameras** are placed at the RealSense **D405** pose read directly from the USD
  (`arm_*_link7/visuals/d405`): local `pos (0.10683, 0, -0.07713)`, 180° about Y.
- The cameras are children of the robot links, so they **follow head/arm motion** at render
  time (the recorded feed tracks the links; only the stage gizmo appears static, which is a
  cosmetic Isaac Sim behaviour).
- **Camera extrinsics are calibrated placeholders** — verify against the physical rig before
  cross-domain training.

> Modified: `pick_place_l_table/joint_pos_env_cfg.py` (camera defs), `pick_place_env_cfg.py`
> (scene slots + observation terms), `mdp/observations.py`.

---

## 5. Drivable Mobile Base **[ADDED]**

The stock `FFW_SG2.usd` is authored for stationary manipulation: a `FixedJoint` welds the
chassis to the world, the wheel drive joints are limited to ±1080°, the left/right wheel
colliders are disabled, and gravity is off. Base motion in the stock task is a kinematic root
teleport, **not** wheel physics.

`FFW_SG2_MOBILE` lifts those locks so the base drives under physics:

| Stock lock | Fix in the drivable variant |
|---|---|
| `FixedJoint` welds chassis to world | `fix_root_link=False` (free base) |
| Wheel drive limit ±1080° (~1.63 m) | Limit removed (continuous rotation) |
| Left/right wheel colliders disabled | Re-enabled (all 3 wheels contact the ground) |
| Gravity off | **Per-body**: on for base + 6 wheel links (traction), off for the arms/lift/head/grippers so they do not sag |
| — | Self-collision **on** (arms cannot pass through the torso), with the 6 wheel links filtered against all body links so wheels collide only with the ground |
| — | Reset event lifts the base to standing height after `reset_scene_to_default` |

The drivable robot is a `~2 KB` override layer that **references** the stock 41 MB USD (the
stock asset is never modified). Verified: settles on its wheels at `root_z ≈ 1.405`, drives
8.23 m in 10 s at 96 % of the commanded speed; holonomic crab / spin-in-place confirmed.

> Added: `assets/robots/FFW_SG2_MOBILE.py`, `data/robots/FFW/FFW_SG2_MOBILE.usd`,
> `controllers/swerve.py` (3-module swerve IK), `scripts/tools/build_ffw_sg2_mobile_usd.py`
> (regenerates the USD), `scripts/tools/check_ffw_sg2_mobile.py` (6-check regression),
> `scripts/tools/teleop_sg2_mobile.py` (keyboard driving).

**Regenerate / verify the drivable USD:**

```bash
# inside the cyclo_lab container, at /workspace/cyclo_lab
./third_party/IsaacLab/isaaclab.sh -p scripts/tools/build_ffw_sg2_mobile_usd.py --force
./third_party/IsaacLab/isaaclab.sh -p scripts/tools/check_ffw_sg2_mobile.py --headless   # expect 6/6
```

---

## 6. VR Teleoperation with Base Driving **[MODIFIED]**

During recording the operator teleoperates the arms **and** drives the base from the Quest
joystick. The base is driven by publishing `/cmd_vel` (a `Twist`) which the sim consumes to
drive the swerve wheels physically (`integrate_root=False`).

### 6.1 Controls

| Input | Action |
|---|---|
| Hold both grips ~3 s | Enable arm teleop (unchanged) |
| **Left Y button** | Toggle base mode: `LIFT+HEAD` ↔ `LIFT+CMD_VEL` **[ADDED]** |
| Left thumbstick (in `CMD_VEL` mode) | Base translate (forward/back, strafe) |
| **A button (hold)** | Turn base **right** (`angular.z < 0`) — works in both modes **[ADDED]** |
| **B button (hold)** | Turn base **left** (`angular.z > 0`) — works in both modes **[ADDED]** |
| Right thumbstick X | Lift up/down |

Notes:
- A + B pressed together cancel (right − left = 0); press one at a time.
- In Vuer's mapping each controller exposes `aButton`/`bButton`; the physical **Y** is the
  **left** controller's `bButton`, and rotation A/B are the **right** controller's buttons.
- A short response delay when starting/changing direction is normal swerve behaviour (the
  wheels re-steer before the base moves), amplified when the sim runs below real time.

### 6.2 The `/cmd_vel` transport fix **[MODIFIED — critical]**

Base commands were being silently dropped by a **DDS QoS mismatch**: the VR publisher sent
`/cmd_vel` as `BEST_EFFORT`, but the sim SDK subscribes `RELIABLE`, and a `RELIABLE` reader
does not match a `BEST_EFFORT` writer. `/cmd_vel` was switched to the `RELIABLE`
`vr_command_qos` (the same profile the working arm/lift/head command topics use).

> Modified: `robotis_applications/robotis_vuer/vr_publisher_sg2.py` (Y toggle, A/B rotation in
> both modes, `/cmd_vel` → RELIABLE), `dds_sdk/ffw_sg2_sdk.py` (subscribe `/cmd_vel`, apply
> swerve base driving each step), `pick_place_l_table/__init__.py` (mobile task registration),
> `mdp/ffw_sg2_l_table_events.py` (`reset_mobile_base_standing`).

Deployment reminder: the VR publisher is symlink-installed, so after editing it, restart the
VR node (dashboard **Launch VR + Controller**) to load the change. See `DEPLOY_PLAN_B.md`.

---

## 7. Recording Demonstrations

### 7.1 Launch

```bash
cd ~/AIWORKER/cyclo_lab
python3 sg2_ltable_dashboard.py          # http://localhost:8765
```

1. Select task **"L-Table Pick & Place (mobile base)"** **[ADDED to the dashboard]**.
2. **Launch Record** → Isaac Sim opens; the robot stands on its wheels.
3. Connect the Quest (USB/ADB recommended) and enter VR.
4. Enter `LIFT+CMD_VEL` mode with the **Y** button to drive the base.

### 7.2 Recording controls (Isaac window focus)

| Key | Action |
|---|---|
| **B** | Start recording + arm teleop for this take |
| **N** | Save the episode |
| **R** | Discard and restart |

### 7.3 Session-based dataset naming **[MODIFIED]**

The dashboard writes datasets under a per-session layout for easier management with
concurrent operators:

```
datasets/{base}/{YYYYMMDD}/{HHMM}_{ip}_{src}_{base}_{stage}.hdf5
#   ip  = last octet of the host IPv4     src = 'vr' | 'leader'     stage = raw|ik|annotate|gen|joint
```

Set `USE_SESSION_NAMING = False` in the dashboard to revert to flat `datasets/<base>_<stage>.hdf5`.

> Modified: `sg2_ltable_dashboard.py`.

---

## 8. LeRobot Conversion **[MODIFIED]**

The converter auto-detects what a recording contains and emits the real-robot schema — no flags
needed:

```bash
# inside cyclo_lab, at /workspace/cyclo_lab
lerobot-python scripts/sim2real/imitation_learning/data_converter/isaaclab2lerobot.py \
  --task Cyclo-Real-Pick-Place-LTable-Mobile-FFW-SG2-v0 \
  --robot_type FFW_SG2 --fps 15 \
  --dataset_file ./datasets/<...>_joint.hdf5
```

What it does:
- **Cameras** — detects the recorded camera streams and exports each as
  `observation.images.rgb.<name>`, **channels-first `[3, H, W]`**, matching the real datasets.
  `cam_head` → `cam_left_head`; resolution is read from the recorded array.
- **Base velocity** — if `obs/base_velocity` is present (mobile task), appends
  `[linear_x, linear_y, angular_z]` to `observation.state` and `action`, taking state/action
  to **22 dims**; otherwise stays 19-dim (stock task unchanged).
- Logs `Base velocity: present -> 22-dim state/action` and the detected cameras.

> Modified: `data_converter/isaaclab2lerobot.py`. Base-velocity observation added in
> `mdp/observations.py` (`base_planar_velocity`) and wired into the mobile task's policy group
> in `joint_pos_env_cfg.py`.

---

## 9. Output Data Schema (matches real `ffw_sg2_rev1`)

| Feature | Shape | Notes |
|---|---|---|
| `observation.state` | (22,) | 19 joints + `linear_x, linear_y, angular_z` |
| `action` | (22,) | same layout (base part = measured twist) |
| `observation.images.rgb.cam_left_head` | [3, 376, 672] | video, CHW |
| `observation.images.rgb.cam_right_head` | [3, 376, 672] | video, CHW |
| `observation.images.rgb.cam_left_wrist` | [3, 424, 240] | video, CHW |
| `observation.images.rgb.cam_right_wrist` | [3, 424, 240] | video, CHW |

The 19 joints: `arm_l_joint1..7, gripper_l_joint1, arm_r_joint1..7, gripper_r_joint1,
head_joint1, head_joint2, lift_joint`.

---

## 10. Known Limitations

- **Mimic / datagen is not yet wired for the mobile 22-dim data** — recording and LeRobot
  conversion are supported; the synthetic-augmentation pipeline (IK convert → annotate →
  datagen) still targets the 19-dim path.
- **Camera extrinsics are placeholders** — calibrate against the physical rig.
- **Frame drops** with 4 cameras on a saturated GPU affect teleop smoothness but **not the
  recorded data** (each frame is the correct image at its simulation timestep).
- **Action base velocity uses the measured twist** as a stand-in for the command (the swerve
  tracks `/cmd_vel` closely).

---

## Appendix A — Files Added / Modified

All paths are under `overlays/` (the version-controlled overlay); they are rsynced onto the
live checkout by `setup.sh` / `sync_overlay.sh`.

### Added

| File | Purpose |
|---|---|
| `cyclo_lab/source/cyclo_lab/cyclo_lab/assets/robots/FFW_SG2_MOBILE.py` | Drivable-base articulation config |
| `cyclo_lab/source/cyclo_lab/data/robots/FFW/FFW_SG2_MOBILE.usd` | ~2 KB override layer over the stock USD |
| `cyclo_lab/source/cyclo_lab/cyclo_lab/controllers/swerve.py` (+ `__init__.py`) | 3-module holonomic swerve IK controller |
| `cyclo_lab/scripts/tools/build_ffw_sg2_mobile_usd.py` | Regenerates `FFW_SG2_MOBILE.usd` |
| `cyclo_lab/scripts/tools/check_ffw_sg2_mobile.py` | Drive/holonomic regression (6 checks) |
| `cyclo_lab/scripts/tools/teleop_sg2_mobile.py` | Keyboard base driving (dev tool) |
| `DEPLOY_PLAN_B.md` | Deployment & test procedure for mobile recording |

### Modified

| File | Change |
|---|---|
| `cyclo_lab/.../pick_place_l_table/joint_pos_env_cfg.py` | 4 cameras; `FFWSG2PickPlaceLTableMobileEnvCfg`; base-velocity obs; reset event |
| `cyclo_lab/.../pick_place_l_table/pick_place_env_cfg.py` | Camera scene slots + observation terms; teleop flags |
| `cyclo_lab/.../pick_place_l_table/__init__.py` | Register the mobile task |
| `cyclo_lab/.../pick_place_l_table/mdp/observations.py` | `base_planar_velocity` observation |
| `cyclo_lab/.../pick_place_l_table/mdp/ffw_sg2_l_table_events.py` | `reset_mobile_base_standing` event |
| `cyclo_lab/scripts/sim2real/imitation_learning/data_converter/isaaclab2lerobot.py` | Auto-detect cameras (rgb, CHW) + base velocity → 22-dim |
| `cyclo_lab/scripts/sim2real/imitation_learning/dds_sdk/ffw_sg2_sdk.py` | Subscribe `/cmd_vel`, physical swerve base driving |
| `cyclo_lab/sg2_ltable_dashboard.py` | Session-based dataset naming; mobile task in the task list |
| `robotis_applications/robotis_vuer/robotis_vuer/vr_publisher_sg2.py` | Y-button mode toggle; A/B base rotation (both modes); `/cmd_vel` → RELIABLE QoS |
