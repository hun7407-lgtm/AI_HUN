# Data Collection Pipeline — FFW-SG2 VR Teleoperation

**Author:** Hun Kim (hun7728@hanyang.ac.kr) · **Last updated:** 2026-07-20

Guide to collecting bimanual manipulation **and mobile-base** demonstrations in NVIDIA Isaac
Sim through Meta Quest VR teleoperation, running the ROBOTIS IsaacLab-Mimic augmentation
pipeline, and exporting to the **LeRobot** format for imitation-learning.

The document follows the order the pipeline runs — **setup → environment → VR connection →
teleoperation → recording → augmentation → conversion** — and covers both the stock (upstream)
behaviour and the changes made in this fork. Work added on top of the upstream stack is tagged
**[ADDED]** / **[MODIFIED]**; every touched file is listed in
[Appendix A](#appendix-a--files-added--modified).

**Repository lineage.** This repo (`AI_HUN`, https://github.com/hun7407-lgtm/AI_HUN) is an
overlay built on top of the base repo **`EKAIWORKER`**
(https://github.com/Disniekie01/EKAIWORKER), which itself pins three upstream ROBOTIS
repositories (`cyclo_lab`, `ai_worker`, `robotis_applications`). Everything under `overlays/`
here is applied on top of those pinned upstreams by `setup.sh`.

---

## Table of Contents

1. [Overview & System Architecture](#1-overview--system-architecture)
2. [Setup](#2-setup)
3. [Simulation Environment & Task](#3-simulation-environment--task)
4. [VR Teleoperation System](#4-vr-teleoperation-system)
5. [Teleoperation Controls](#5-teleoperation-controls)
6. [Recording a Demonstration](#6-recording-a-demonstration)
7. [IsaacLab-Mimic Augmentation Pipeline](#7-isaaclab-mimic-augmentation-pipeline)
8. [LeRobot Conversion & Output Schema](#8-lerobot-conversion--output-schema)
9. [Real-Robot Parity](#9-real-robot-parity)
10. [The Drivable Mobile Base](#10-the-drivable-mobile-base)
11. [Known Limitations](#11-known-limitations)
12. [Appendix A — Files Added / Modified](#appendix-a--files-added--modified)

---

## 1. Overview & System Architecture

### 1.1 Goal

Collect demonstrations in simulation whose **format matches the physical `ffw_sg2_rev1`
robot**, so simulated and real demonstrations can be pooled to train the same policy. A single
demonstration captures, per timestep: the robot's joint state, the base velocity, the commanded
action, and four RGB camera streams.

### 1.2 Robot

ROBOTIS **FFW-SG2** — a dual-arm mobile manipulator:

| Subsystem | DoF | Notes |
|---|---|---|
| Left arm | 7 | `arm_l_joint1..7` |
| Left gripper | 1 driven (+3 mimic) | `gripper_l_joint1` driven; `2..4` follow via a PhysX mimic linkage |
| Right arm | 7 | `arm_r_joint1..7` |
| Right gripper | 1 driven (+3 mimic) | `gripper_r_joint1` |
| Head | 2 | `head_joint1` (pitch), `head_joint2` (yaw) |
| Lift | 1 | `lift_joint` (vertical torso lift, prismatic) |
| Base | 3 swerve modules | left / right / rear, each steer + drive |

The **action / joint state is 19-dim** (7+1 + 7+1 + 2 + 1). On the mobile task it is **22-dim**
(+ base `linear_x, linear_y, angular_z`).

### 1.3 Task

**L-table pick & place:** grasp a cardboard box on the front table with both grippers, carry
it, and place it on the left ("L") table. The base must reposition between the pick and place
locations.

### 1.4 The three containers

Data collection runs across three Docker containers that talk over ROS 2 (`ROS_DOMAIN_ID=30`,
RMW = `rmw_fastrtps_cpp`, i.e. Fast DDS):

| Container | Image | Role | Ports |
|---|---|---|---|
| `cyclo_lab` | `cyclolab/cyclo-lab:latest` (Isaac Sim 5.1.0 base) | Isaac Sim, task env, recorder, Mimic pipeline, **dashboard** | 8765 |
| `robotis-applications` | `robotis/robotis-applications:1.0.0` | **Vuer VR publisher** (`vr_publisher_sg2`) | 8012 |
| `ai_worker` | `robotis/ai-worker:2.0.0` | **arm IK controller** (`vr_controller`) | — |

### 1.5 Data flow

```
[Operator + Meta Quest 3]
   │  hand/controller pose + joystick  (WebXR)
   ▼
[robotis-applications]  Vuer VR publisher
   ├── arm/wrist poses ─(DDS)─►  [ai_worker] vr_controller ── IK ── joint commands ─(DDS)─┐
   └── /cmd_vel (Twist) ───────────────────────────────────────────────────────────────┐ │
                                                                                        ▼ ▼
[cyclo_lab]  Isaac Sim + record_demos.py + FFWSG2Sdk
   │   • applies arm joint commands to the sim robot
   │   • drives the swerve base from /cmd_vel  (mobile task)
   │   • renders 4 cameras + reads joint/base state
   ▼
  raw .hdf5
   │
   ├─► IK convert ─► annotate ─► IsaacLab-Mimic datagen ─► joint convert ─► LeRobot export
   ▼
LeRobot dataset  (4 cameras + 22-dim state/action)
```

---

## 2. Setup

### 2.1 Prerequisites (not installed by `setup.sh`)

| Requirement | Verify |
|---|---|
| Linux host (Ubuntu / Pop!_OS) + X11 | — |
| NVIDIA RTX GPU + driver (validated: RTX PRO 5000, 24 GB) | `nvidia-smi` |
| Docker Engine + NVIDIA Container Toolkit | `docker run --gpus all nvidia/cuda:12.4.0-base-ubuntu22.04 nvidia-smi` |
| NGC login (pulls `nvcr.io/nvidia/isaac-sim:5.1.0`) | `docker login nvcr.io` |
| Meta Quest 3 (+ one-time ADB/udev for USB tether) | `adb_vr_connect/README.md` |
| Disk (images + datasets) | hundreds of GB |

### 2.2 Clone and build

```bash
git clone https://github.com/hun7407-lgtm/AI_HUN.git AIWORKER
cd AIWORKER
./setup.sh ~/AIWORKER
```

`setup.sh` does, in order:
1. `git clone` the three upstream ROBOTIS repos at **pinned commits**
   (`cyclo_lab`, `ai_worker`, `robotis_applications`).
2. `git submodule update --init --recursive` for `cyclo_lab` (pulls Isaac Lab into
   `cyclo_lab/third_party/IsaacLab`).
3. `rsync` the fork's `overlays/cyclo_lab/` and `overlays/robotis_applications/` on top.

Result: `~/AIWORKER/{cyclo_lab, ai_worker, robotis_applications}`. To re-apply overlay edits to
a live checkout later, use `./sync_overlay.sh ~/AIWORKER/cyclo_lab` (overlay → live) or
`./pull_overlay.sh` (live → overlay).

### 2.3 Start containers and enable the GUI

```bash
cd ~/AIWORKER/cyclo_lab/docker            && ./container.sh start
cd ~/AIWORKER/robotis_applications/docker && ./container.sh start
cd ~/AIWORKER/ai_worker/docker            && ./container.sh start

# host: allow the containers to open GUI windows on the local X server
xhost +local:docker
xhost +SI:localuser:root      # (and `xhost +` if the Isaac window fails to appear)
```

---

## 3. Simulation Environment & Task

### 3.1 Isaac Sim + Isaac Lab

- **Isaac Sim 5.1.0** — physics + rendering engine (base container image).
- **Isaac Lab 2.3.0** — the task framework: scene definition, the manager system
  (observations / actions / events / terminations / recorder), and the IsaacLab-Mimic dataset
  tooling. Sub-packages: `isaaclab 0.47.2`, `isaaclab_mimic 1.0.15`, `isaaclab_tasks 0.11.6`,
  `isaaclab_rl 0.4.4`, `isaaclab_assets 0.2.3`.

### 3.2 The L-table task, as a manager-based RL env

The task is defined as an Isaac Lab `ManagerBasedRLEnvCfg` (`pick_place_env_cfg.py` +
`joint_pos_env_cfg.py`). The managers relevant to data collection:

**Scene** — the robot, the front/left tables, the cardboard box, a box riser, a drop-zone
marker, ground/light, and the cameras.

**Actions (19-dim total)** — six action terms:

| Term | Joints | dim |
|---|---|---|
| `arm_l_action` | `arm_l_joint1..7` | 7 |
| `gripper_l_action` | `gripper_l_joint1` | 1 |
| `arm_r_action` | `arm_r_joint1..7` | 7 |
| `gripper_r_action` | `gripper_r_joint1` | 1 |
| `head_action` | `head_joint1..2` | 2 |
| `lift_action` | `lift_joint` | 1 |

**Observations (`policy` group)** — recorded per timestep:

| Term | Content |
|---|---|
| `actions` | last action |
| `joint_pos` | 19 joint positions |
| `joint_pos_target` | 19 joint targets |
| `left_eef_pose` / `right_eef_pose` | end-effector poses (frame transformers) |
| `cam_head`, `cam_right_head`, `cam_left_wrist`, `cam_right_wrist` | RGB images **[MODIFIED — see §9]** |
| `base_velocity` | `[linear_x, linear_y, angular_z]` (mobile task only) **[ADDED]** |

**Recorder** — `ActionStateRecorderManagerCfg` writes each episode's actions + observations +
initial state into an HDF5 group `data/demo_<i>`.

**Terminations** — `time_out`, task `success` (box on the left table), `object_dropped`.

### 3.3 Available tasks

Six SG2 tasks are selectable in the dashboard. All but the mobile one use a **fixed
(welded) base**:

| # | Dashboard label | Task ID | Base |
|---|---|---|---|
| 1 | Basket Pick & Place | `Cyclo-Real-Pick-Place-FFW-SG2-v0` | fixed |
| 2 | L-Table Pick & Place (thin box) | `Cyclo-Real-Pick-Place-LTable-FFW-SG2-v0` | fixed |
| 3 | **L-Table Pick & Place (mobile base)** **[ADDED]** | `Cyclo-Real-Pick-Place-LTable-Mobile-FFW-SG2-v0` | **free (drivable)** |
| 4 | Box Stack (thick box) | `Cyclo-Real-Box-Stack-FFW-SG2-v0` | fixed |
| 5 | Single Box Far (rear table) | `Cyclo-Real-Single-Box-Far-FFW-SG2-v0` | fixed |
| 6 | Single Box Far (thick box) | `Cyclo-Real-Single-Box-Far-Thick-FFW-SG2-v0` | fixed |

The **mobile (drivable) base is applied only to the L-Table Pick & Place task** (row 3). Every
task also has a `…-Mimic-…` env used by the augmentation pipeline (§7) and a `FFW_SH5` (hand)
counterpart. The rest of this document uses the L-Table task as the running example, but the
recording/augmentation/conversion flow is the same for the other fixed-base tasks.

| L-Table variant | Base motion | dims |
|---|---|---|
| `…-Pick-Place-LTable-FFW-SG2-v0` (fixed) | scripted L-motion (kinematic teleport) | 19 |
| `…-Pick-Place-LTable-Mobile-FFW-SG2-v0` **[ADDED]** | operator drives via `/cmd_vel` (physical swerve) | 22 |

---

## 4. VR Teleoperation System

### 4.1 Vuer / WebXR

The operator wears a Meta Quest 3 and opens **Vuer** (a WebXR page) in the Quest browser. Vuer
streams the headset/controller poses and button states to the PC over a WebSocket; no app
install is needed. The `robotis_vuer` node (`vr_publisher_sg2`) serves Vuer on port **8012**.

### 4.2 The publisher → controller → sim chain

1. **`vr_publisher_sg2`** (`robotis-applications`) receives the WebXR stream and publishes, over
   ROS 2 / DDS: wrist/elbow/shoulder poses, gripper squeeze, lift/head joystick commands, and —
   for the base — **`/cmd_vel`** (a `Twist`).
2. **`vr_controller`** (`ai_worker`) subscribes to the wrist poses and solves inverse
   kinematics, publishing arm joint trajectories.
3. **`FFWSG2Sdk`** (`cyclo_lab`, `dds_sdk/ffw_sg2_sdk.py`) subscribes to the arm/lift/head
   trajectories and applies them to the sim robot; it also subscribes to **`/cmd_vel`** and
   drives the swerve base (mobile task) **[MODIFIED — see §10]**.

All three share `ROS_DOMAIN_ID=30`. Command topics (arms, lift, head, and `/cmd_vel`) use a
**RELIABLE** QoS so DDS delivers them to the SDK's reliable readers.

### 4.3 Connecting the Quest

There are two ways to connect the headset. **USB is recommended** — it has lower latency
jitter than WiFi (WiFi spikes show up as arm/base stutter during teleop).

**USB / ADB tether (recommended):**
```bash
cd ~/AIWORKER/adb_vr_connect && ./connect.sh     # re-run after every cable reconnect
# Quest browser:  https://localhost:8012?ws=wss://localhost:8012  → accept cert → Enter VR
```
One-time ADB/udev setup: `adb_vr_connect/README.md`.

**WiFi:** two steps in the Quest browser — accept the cert at `https://<PC-IP>:8012`, then open
`https://vuer.ai/?ws=wss://<PC-IP>:8012` and Enter VR. The full WiFi procedure is documented in
the base repo's README:
[`EKAIWORKER`](https://github.com/Disniekie01/EKAIWORKER) → *VR Teleoperation — How to Connect*.

The general VR teleoperation setup (Vuer, headset, publisher/controller) follows the ROBOTIS
guide: <https://docs.robotis.com/docs/systems/aiworker/quick_start_guide/operation_guide/vr_teleoperation/>.

---

## 5. Teleoperation Controls

| Input | Action |
|---|---|
| **Hold both grips ~3 s** | Enable arm teleop (and base cmd_vel) for the session |
| Move hands in VR | Arm end-effectors follow (IK); squeeze to close grippers |
| Right thumbstick X | Lift up / down |
| **Left Y button** | Toggle base mode: `LIFT+HEAD` ↔ `LIFT+CMD_VEL` **[ADDED]** |
| Left thumbstick (in `CMD_VEL` mode) | Base translate (forward/back, strafe) |
| **A button (hold)** | Turn base right (`angular.z < 0`) — both modes **[ADDED]** |
| **B button (hold)** | Turn base left (`angular.z > 0`) — both modes **[ADDED]** |
| Left thumbstick (in `LIFT+HEAD` mode) | Head pan/tilt |

Notes: A + B pressed together cancel (press one at a time). In Vuer's mapping each controller
exposes `aButton`/`bButton`, so the physical **Y** is the left controller's `bButton`. A short
start/turn delay is normal swerve behaviour (wheels re-steer before the base moves).

---

## 6. Recording a Demonstration

### 6.1 Launch the stack

```bash
cd ~/AIWORKER/cyclo_lab
python3 sg2_ltable_dashboard.py            # http://localhost:8765
```

On the dashboard:
1. Select the task — e.g. **"L-Table Pick & Place (mobile base)"** **[ADDED to the dashboard]**.
2. **Launch Record** — starts Isaac Sim (with the recorder) + the VR publisher + the controller.
   Wait until it shows `vr: running` and `ai: running`.
3. Connect the Quest (§4.3) and enter VR.

### 6.2 Per-take recording cycle (Isaac window focus)

| Key | Action |
|---|---|
| **B** | Start recording + arm teleop for this take (press before every take) |
| **N** | Save the episode |
| **R** | Discard the take and restart |

On the mobile task, drive the base with the joystick (§5) while manipulating. On success (box
placed) the recorder can auto-save if configured.

### 6.3 What is recorded, and where

Each saved episode becomes an HDF5 group:

```
data/demo_<i>/
  initial_state/…                 # articulation + rigid-object poses at reset
  actions            (T, 19|22)   # per-step action
  obs/joint_pos      (T, 19)      # joint positions
  obs/joint_pos_target (T, 19)
  obs/base_velocity  (T, 3)       # mobile task only  [ADDED]
  obs/cam_head       (T, 376, 672, 3)   uint8 HWC     [MODIFIED]
  obs/cam_right_head (T, 376, 672, 3)                 [ADDED]
  obs/cam_left_wrist (T, 424, 240, 3)                 [ADDED]
  obs/cam_right_wrist(T, 424, 240, 3)                 [ADDED]
  attrs: success, seed, num_samples
```

**Session-based dataset naming [MODIFIED]** — the dashboard writes:
```
datasets/{base}/{YYYYMMDD}/{HHMM}_{ip}_{src}_{base}_{stage}.hdf5
#  ip = host IPv4 last octet   src = vr|leader   stage = raw|ik|annotate|gen|joint
```
Set `USE_SESSION_NAMING = False` in the dashboard to revert to flat `datasets/<base>_<stage>.hdf5`.

The `raw` file from recording is the input to the augmentation pipeline (§7).

---

## 7. IsaacLab-Mimic Augmentation Pipeline

A handful of teleoperated demonstrations is expanded into hundreds of training demonstrations
with **IsaacLab-Mimic**, run one step at a time from the dashboard **Mimic pipeline** section or
manually inside the `cyclo_lab` container. The five stages (SG2 L-table example):

| # | Stage | Script | In → Out | What it does |
|---|---|---|---|---|
| 1 | **IK convert** | `action_data_converter.py --action_type ik` | `raw → ik` | Converts recorded **joint** actions to **end-effector (IK)** actions, the representation Mimic recombines. |
| 2 | **Annotate** | `annotate_demos.py --auto` | `ik → annotate` | Labels each demo's **subtask segments** (grasp / move / place) so Mimic knows where it can splice. |
| 3 | **Datagen** | `generate_dataset.py` | `annotate → generate` | IsaacLab-Mimic **synthesizes** new demos: it replays and recombines the annotated subtasks under randomized box/table/lighting, across `--num_envs` parallel Isaac environments, until `--generation_num_trials` successes are produced (SG2 L-table uses `cyclo_mimic_datagen.py` for the lift/head + base motion). |
| 4 | **Joint convert** | `action_data_converter.py --action_type joint` | `generate → joint` | Converts the generated IK actions back to **joint** actions for training. |
| 5 | **LeRobot export** | `isaaclab2lerobot.py` | `joint → lerobot` | Exports to the LeRobot dataset (§8). |

Run stages **one at a time**, waiting for each to finish. Manual commands (inside the
`cyclo_lab` container, at `/workspace/cyclo_lab`):

```bash
RAW=./datasets/ffw_sg2_l_table_raw.hdf5
IK=./datasets/ffw_sg2_l_table_ik.hdf5

# 1) IK convert
python scripts/sim2real/imitation_learning/mimic/action_data_converter.py \
  --robot_type FFW_SG2 --input_file "$RAW" --output_file "$IK" --action_type ik

# 2) Annotate
python scripts/sim2real/imitation_learning/mimic/annotate_demos.py \
  --task Cyclo-Real-Mimic-Pick-Place-LTable-FFW-SG2-v0 --auto \
  --input_file "$IK" --output_file ./datasets/ffw_sg2_l_table_annotate.hdf5 --enable_cameras --headless

# 3) Datagen (500 synthetic demos, 10 parallel envs)
python scripts/sim2real/imitation_learning/mimic/generate_dataset.py \
  --device cuda --num_envs 10 --task Cyclo-Real-Mimic-Pick-Place-LTable-FFW-SG2-v0 \
  --generation_num_trials 500 --input_file ./datasets/ffw_sg2_l_table_annotate.hdf5 \
  --output_file ./datasets/ffw_sg2_l_table_generate.hdf5 --enable_cameras --headless

# 4) Joint convert
python scripts/sim2real/imitation_learning/mimic/action_data_converter.py \
  --robot_type FFW_SG2 --input_file ./datasets/ffw_sg2_l_table_generate.hdf5 \
  --output_file ./datasets/ffw_sg2_l_table_joint.hdf5 --action_type joint
```

Tuning env vars (before starting the dashboard): `GENERATION_NUM_TRIALS` (default 500),
`PIPELINE_NUM_ENVS` (default 10). Per-step logs: `docker exec cyclo_lab tail -f
/tmp/sg2_ltable_pipe_generate.log`.

> Note: the Mimic path currently targets the **19-dim** (arms-only) data. Mobile 22-dim data is
> supported through recording and LeRobot export, but IK/annotate/datagen are not yet wired for
> the base-velocity dimensions (see §11).

---

## 8. LeRobot Conversion & Output Schema

### 8.1 The converter **[MODIFIED]**

`isaaclab2lerobot.py` auto-detects what a recording contains and emits the real-robot schema —
no flags needed:

```bash
lerobot-python scripts/sim2real/imitation_learning/data_converter/isaaclab2lerobot.py \
  --task Cyclo-Real-Pick-Place-LTable-Mobile-FFW-SG2-v0 \
  --robot_type FFW_SG2 --fps 15 \
  --dataset_file ./datasets/<...>_joint.hdf5
```

- **Cameras** — detects the recorded camera streams and exports each as
  `observation.images.rgb.<name>`, **channels-first `[3, H, W]`** (matching real). `cam_head` →
  `cam_left_head`; resolution is read from the recorded array.
- **Base velocity** — if `obs/base_velocity` is present, appends `[linear_x, linear_y,
  angular_z]` to `observation.state` and `action`, giving **22 dims**; otherwise **19 dims**.
- Logs the detected cameras and `Base velocity: present -> 22-dim state/action`.

### 8.2 Output schema (matches real `ffw_sg2_rev1`)

| Feature | Shape | Notes |
|---|---|---|
| `observation.state` | (22,) | 19 joints + `linear_x, linear_y, angular_z` |
| `action` | (22,) | same layout |
| `observation.images.rgb.cam_left_head` | [3, 376, 672] | video, CHW |
| `observation.images.rgb.cam_right_head` | [3, 376, 672] | video, CHW |
| `observation.images.rgb.cam_left_wrist` | [3, 424, 240] | video, CHW |
| `observation.images.rgb.cam_right_wrist` | [3, 424, 240] | video, CHW |

19 joints, in order: `arm_l_joint1..7, gripper_l_joint1, arm_r_joint1..7, gripper_r_joint1,
head_joint1, head_joint2, lift_joint`.

---

## 9. Real-Robot Parity

The real `ffw_sg2_rev1` records **4** RGB cameras and a **22-dim** state; the stock sim recorded
1 camera and 19 dims. Two additions match the real schema.

### 9.1 Cameras **[MODIFIED]**

| LeRobot key | Sim camera | Mount | Resolution |
|---|---|---|---|
| `…rgb.cam_left_head` | `cam_head` (ZED left eye) | `head_link2/zed` | 376 × 672 |
| `…rgb.cam_right_head` | `cam_right_head` | `head_link2/zed`, mirrored | 376 × 672 |
| `…rgb.cam_left_wrist` | `cam_left_wrist` | `arm_l_link7` (D405 pose) | 424 × 240 |
| `…rgb.cam_right_wrist` | `cam_right_wrist` | `arm_r_link7` (D405 pose) | 424 × 240 |

- Head cameras are the two eyes of the head ZED, symmetric about the `zed` prim centre (±0.03 m
  in Y ⇒ ~0.06 m baseline).
- Wrist cameras use the RealSense **D405** pose read from the USD
  (`arm_*_link7/visuals/d405`): local `pos (0.10683, 0, -0.07713)`, 180° about Y.
- Cameras are children of the robot links, so the **rendered/recorded feed follows head/arm
  motion** (the stage gizmo may look static — cosmetic only).
- **Extrinsics are calibrated placeholders** — verify against the physical rig before
  cross-domain training.

### 9.2 Base velocity **[ADDED]**

The mobile task records `obs/base_velocity = base_planar_velocity(env)` =
`[root_lin_vel_b.x, root_lin_vel_b.y, root_ang_vel_b.z]` (base frame). The converter appends it
to state and action → 22-dim, matching real.

---

## 10. The Drivable Mobile Base **[ADDED]**

### 10.1 Why a drivable base was added

On the fixed-base (base-repo) form, the base repositions by a **scripted L-motion that
kinematically teleports the robot root**. Two problems come from this:
- The teleport fights the physics simulation — while the root is teleported, the carried box is
  still driven by physics, so it **shakes / jitters** relative to the grippers.
- The kinematic-teleport base motion did **not carry cleanly through data generation**
  (IsaacLab-Mimic), so mobile-base trajectories could not be augmented reliably.

To avoid the teleport-vs-physics conflict, a **physically drivable** base USD (`FFW_SG2_MOBILE`)
was created: the base moves by actually driving its swerve wheels under physics, with no root
teleport, so the base and the carried box stay physically consistent.

### 10.2 What changed in the USD

The stock `FFW_SG2.usd` is authored for stationary manipulation: a `FixedJoint` welds the
chassis to the world, the wheel drive joints stop at ±1080°, the left/right wheel colliders are
disabled, and gravity is off. Its base "moves" by kinematic root teleport, not wheel physics.

`FFW_SG2_MOBILE` lifts those locks in a `~2 KB` override layer that references the stock USD
(the stock asset is never modified):

| Stock lock | Fix |
|---|---|
| `FixedJoint` welds chassis to world | `fix_root_link=False` (free base) |
| Wheel drive limit ±1080° | removed (continuous rotation) |
| Left/right wheel colliders off | re-enabled |
| Gravity off | **per-body**: on for base + 6 wheels (traction), off for arms/lift/head/grippers (no sag) |
| — | self-collision **on** (arms can't pass through torso); 6 wheel links filtered vs all body links (wheels touch only the ground) |
| — | a reset event lifts the base to standing height after `reset_scene_to_default` |

During recording, the operator drives the base from `/cmd_vel`; the SDK applies it as swerve
wheel targets (`_apply_swerve_cmd_vel(..., integrate_root=False)` → physical driving). Verified:
settles at `root_z ≈ 1.405`, drives 8.23 m in 10 s at 96 % of the commanded speed; holonomic
crab / spin-in-place confirmed.

Tools: `scripts/tools/build_ffw_sg2_mobile_usd.py` (regenerate the USD),
`check_ffw_sg2_mobile.py` (6/6 regression), `teleop_sg2_mobile.py` (keyboard driving).

---

## 11. Known Limitations

- **Mimic / datagen is not yet wired for the mobile 22-dim data.** Recording and LeRobot export
  handle 22 dims; the augmentation pipeline (IK convert → annotate → datagen) still targets the
  19-dim path.
- **Camera extrinsics are placeholders** — calibrate against the physical rig.
- **Rendering four cameras is GPU-heavy.** On the validated workstation
  (**NVIDIA RTX PRO 5000 Blackwell Laptop, 24 GB**) frame drops do not occur in normal use — any
  drops observed were transient. On other / lower-spec workstations, rendering four cameras may
  saturate the GPU and cause frame drops, which slow teleop. Note that frame drops affect only
  teleop smoothness, **not the recorded data**: each frame is still the correct image at its
  simulation timestep; the sim just runs slower than real time. (If needed on a weaker machine,
  run the recorder headless to drop the GUI viewport render — no effect on the data.)
- **The action's base velocity uses the measured twist** as a stand-in for the command (the
  swerve tracks `/cmd_vel` closely).

---

## Appendix A — Files Added / Modified

All paths are under `overlays/` (the version-controlled overlay), applied onto the live
checkout by `setup.sh` / `sync_overlay.sh`.

### Added

| File | Purpose |
|---|---|
| `cyclo_lab/…/assets/robots/FFW_SG2_MOBILE.py` | Drivable-base articulation config |
| `cyclo_lab/…/data/robots/FFW/FFW_SG2_MOBILE.usd` | ~2 KB override layer over the stock USD |
| `cyclo_lab/…/controllers/swerve.py` (+ `__init__.py`) | 3-module holonomic swerve IK controller |
| `cyclo_lab/scripts/tools/build_ffw_sg2_mobile_usd.py` | Regenerates `FFW_SG2_MOBILE.usd` |
| `cyclo_lab/scripts/tools/check_ffw_sg2_mobile.py` | Drive / holonomic regression (6 checks) |
| `cyclo_lab/scripts/tools/teleop_sg2_mobile.py` | Keyboard base driving (dev tool) |
| `DEPLOY_PLAN_B.md` | Deployment & test procedure for mobile recording |

### Modified

| File | Change |
|---|---|
| `cyclo_lab/…/pick_place_l_table/joint_pos_env_cfg.py` | 4 cameras; `FFWSG2PickPlaceLTableMobileEnvCfg`; base-velocity obs; reset event |
| `cyclo_lab/…/pick_place_l_table/pick_place_env_cfg.py` | Camera scene slots + observation terms; teleop flags |
| `cyclo_lab/…/pick_place_l_table/__init__.py` | Register the mobile task |
| `cyclo_lab/…/pick_place_l_table/mdp/observations.py` | `base_planar_velocity` observation |
| `cyclo_lab/…/pick_place_l_table/mdp/ffw_sg2_l_table_events.py` | `reset_mobile_base_standing` event |
| `cyclo_lab/scripts/…/data_converter/isaaclab2lerobot.py` | Auto-detect cameras (rgb, CHW) + base velocity → 22-dim |
| `cyclo_lab/scripts/…/dds_sdk/ffw_sg2_sdk.py` | Subscribe `/cmd_vel`, physical swerve base driving |
| `cyclo_lab/sg2_ltable_dashboard.py` | Session-based dataset naming; mobile task in the task list |
| `robotis_applications/robotis_vuer/robotis_vuer/vr_publisher_sg2.py` | Y-button mode toggle; A/B base rotation (both modes); `/cmd_vel` → RELIABLE QoS |

---

*Base repository: [`EKAIWORKER`](https://github.com/Disniekie01/EKAIWORKER) · Fork:
[`AI_HUN`](https://github.com/hun7407-lgtm/AI_HUN). Upstream: ROBOTIS `cyclo_lab`, `ai_worker`,
`robotis_applications` (pinned in `setup.sh`).*
