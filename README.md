# AIWORKER

Portable overlay for AI Worker Isaac Sim VR teleoperation tasks.

This repo is intentionally small. It does not vendor the full ROBOTIS repositories and it does not include recorded datasets. Instead, `setup.sh` clones the required upstream repos at pinned commits, initializes `cyclo_lab` submodules, and copies the AIWORKER overlay files on top.

**SG2 L-table data + checkpoint:** [Release sg2-ltable-20260702](https://github.com/Disniekie01/EKAIWORKER/releases/tag/sg2-ltable-20260702)

## 5-Minute Quickstart (Fresh Machine)

Use this if you just cloned and want a runnable state quickly.

### 1) Clone + setup

```bash
git clone https://github.com/Disniekie01/EKAIWORKER.git AIWORKER
cd AIWORKER
./setup.sh ~/AIWORKER
```

### 2) Start containers

```bash
cd ~/AIWORKER/cyclo_lab/docker && ./container.sh start
cd ~/AIWORKER/robotis_applications/docker && ./container.sh start
cd ~/AIWORKER/ai_worker/docker && ./container.sh start
```

### 3) Enable GUI (host terminal)

```bash
xhost +local:docker
xhost +SI:localuser:root
```

### 4) Start dashboard

```bash
cd ~/AIWORKER/cyclo_lab
python3 sg2_ltable_dashboard.py
```

Open: `http://localhost:8765`

Select a task, click **Launch VR + Controller**, then connect the headset — see [VR Teleoperation](#vr-teleoperation--how-to-connect) for the full Vuer URL (`https://<host-ip>:8012?ws=wss://<host-ip>:8012`).

### 5) Run first SG2 L-table play test

```bash
docker exec -e DISPLAY=:1 -e TERM=xterm cyclo_lab bash -lc '
cd /workspace/cyclo_lab
./third_party/IsaacLab/isaaclab.sh -p scripts/imitation_learning/robomimic/play.py \
  --device cuda \
  --task Cyclo-Real-Pick-Place-LTable-FFW-SG2-v0 \
  --checkpoint /PATH/TO/model_epoch_20.pth \
  --num_rollouts 3 --horizon 2000 \
  --enable_cameras --action_mode inference --scripted_l_motion
'
```

If GUI does not appear, run:

```bash
xhost +
```

Then retry the play command.

## End-to-End: Record -> Pipeline -> Train -> Play

Use this exact flow for a new task run (example: SG2 L-table).

### 1) Record demos (dashboard)

Start the dashboard:

```bash
cd ~/AIWORKER/cyclo_lab
python3 sg2_ltable_dashboard.py
```

Open `http://localhost:8765`, select your task, launch the stack, connect VR (see [VR Teleoperation](#vr-teleoperation--how-to-connect)), and record demos.

Recorder controls in Isaac window:
- `B` start recording
- `N` save episode
- `R` reset / skip episode
- `L` trigger L-motion

Raw dataset is written to `~/AIWORKER/cyclo_lab/datasets/*_raw.hdf5`.

### 2) Run Mimic pipeline (dashboard)

From the dashboard **Mimic pipeline** section, run steps in order:
1. IK convert (`raw -> ik`)
2. Annotate (`ik -> annotate`)
3. Generate (`annotate -> generate`)
4. Joint convert (`generate -> joint`)
5. (Optional) LeRobot export (`joint -> lerobot`)

Wait for each step to complete before starting the next.

### 3) Train robomimic

```bash
docker exec -it cyclo_lab bash -lc '
cd /workspace/cyclo_lab
./third_party/IsaacLab/isaaclab.sh -p scripts/imitation_learning/robomimic/train.py \
  --task Cyclo-Real-Pick-Place-LTable-FFW-SG2-v0 \
  --algo bc \
  --dataset ./datasets/ffw_sg2_l_table_joint.hdf5 \
  --name ffw_sg2_l_table_bc
'
```

### 4) Play checkpoint

```bash
docker exec -e DISPLAY=:1 -e TERM=xterm -it cyclo_lab bash -lc '
cd /workspace/cyclo_lab
./third_party/IsaacLab/isaaclab.sh -p scripts/imitation_learning/robomimic/play.py \
  --device cuda \
  --task Cyclo-Real-Pick-Place-LTable-FFW-SG2-v0 \
  --checkpoint /workspace/cyclo_lab/logs/robomimic/Cyclo-Real-Pick-Place-LTable-FFW-SG2-v0/ffw_sg2_l_table_bc/<run_id>/models/model_epoch_20.pth \
  --num_rollouts 3 --horizon 2000 \
  --enable_cameras --action_mode inference --scripted_l_motion
'
```

## What This Adds

- `cyclo_lab` dashboard for launching the stack, selecting tasks, and running the Mimic pipeline step-by-step.
- SG2 basket pick-place, L-table, box-stack, single-box-far, and thick-box variants (record + Mimic for SG2).
- SH5 hand versions of the same tasks (record + Mimic).
- SH5 DDS recorder support for VR hand teleoperation.
- Task-specific table/box assets and teleop motion settings.
- Minor `robotis_applications` VR publisher update used by this setup.
- `adb_vr_connect/` — Meta Quest 3 USB tethering via ADB reverse port forwarding (lower jitter than WiFi).

## Upstream Pins

- `cyclo_lab`: `a5ea01967b145f839ca1ac8f51b42abf9ef87036`
- `ai_worker`: `e8c2eacb612e47473cdf03e44bee6d527c00b4f9`
- `robotis_applications`: `7ef0aabc748174cb91013866b2e4142122ef475c`

## Install On A New Machine

```bash
git clone https://github.com/Disniekie01/EKAIWORKER.git AIWORKER
cd AIWORKER
./setup.sh ~/AIWORKER
```

The install directory can be any path. The command above creates:

```text
~/AIWORKER/
  cyclo_lab/
  ai_worker/
  robotis_applications/
```

## Start Containers

```bash
cd ~/AIWORKER/cyclo_lab/docker
./container.sh start

cd ~/AIWORKER/robotis_applications/docker
./container.sh start

cd ~/AIWORKER/ai_worker/docker
./container.sh start
```

## Start Dashboard

```bash
cd ~/AIWORKER/cyclo_lab
python3 sg2_ltable_dashboard.py
```

Open the dashboard at:

```text
http://localhost:8765
```

**Paste this in Meta Quest Browser (WiFi, same LAN as PC):**

```text
https://<host-ip>:8012?ws=wss://<host-ip>:8012
```

Replace `<host-ip>` with your PC IP (dashboard prints the full URL after **Launch VR + Controller**). For USB tethering use `https://localhost:8012?ws=wss://localhost:8012` — see [VR Teleoperation](#vr-teleoperation--how-to-connect).

Select the task, then launch the stack from the dashboard.

### Mimic pipeline (dashboard)

After recording a raw `.hdf5` for the selected task, use the **Mimic pipeline** section on the dashboard (`http://localhost:8765`). Run steps **one at a time** in order and wait for each to finish before starting the next:

1. **IK convert** — `*_raw.hdf5` → `*_ik.hdf5`
2. **Annotate** — `*_ik.hdf5` → `*_annotate.hdf5`
3. **Datagen** — `*_annotate.hdf5` → `*_generate.hdf5` (uses `cyclo_mimic_datagen.py` for lift/head + L-motion replay)
4. **Joint convert** — `*_generate.hdf5` → `*_joint.hdf5`
5. **LeRobot export** — `*_joint.hdf5` → LeRobot dataset

Each button shows its status (`stopped` / `starting` / `running`) and whether the input file exists. Only one pipeline step runs at a time. **Run all steps sequentially** chains the five steps with waits between them.

Pipeline jobs run headless inside the `cyclo_lab` container. Start that container first. Per-step logs:

```bash
docker exec cyclo_lab tail -f /tmp/sg2_ltable_pipe_ik.log
# ik | annotate | generate | joint | lerobot
```

Optional tuning via environment variables before starting the dashboard:

```bash
export GENERATION_NUM_TRIALS=500   # datagen episode count (default 500)
export PIPELINE_NUM_ENVS=10        # parallel Isaac envs during datagen (default 10)
```

HDF5 paths are derived from the task raw dataset name (e.g. `ffw_sg2_l_table_raw.hdf5` → `ffw_sg2_l_table_ik.hdf5`, etc.). The dashboard uses each task's `mimic_id` for annotate/datagen and the record task id for LeRobot export.

## VR Teleoperation — How to Connect

VR teleop uses **Vuer** (WebXR) in the Meta Quest browser. The dashboard starts the full stack; you open Vuer on the headset and control the sim robot from there.

### URLs and ports

| Service | Address | Purpose |
|---------|---------|---------|
| **Dashboard** | `http://localhost:8765` | Task picker, launch VR/controller/recorder, mimic pipeline |
| **Vuer (HTTPS)** | `https://<host-ip>:8012` | WebXR page served by `robotis_vuer` |
| **Vuer (WebSocket)** | `wss://<host-ip>:8012` | Pose/button stream (must use `wss://` with `https://`) |

Replace `<host-ip>` with your PC’s LAN address. The dashboard prints it after **Launch VR + Controller** or **Launch Record** (first address from `hostname -I`, e.g. `192.168.1.42`).

**Full URL to open in Meta Quest Browser (WiFi / same LAN):**

```text
https://<host-ip>:8012?ws=wss://<host-ip>:8012
```

This is the address you need — both the page **and** the `ws=` parameter must use the same host (`<host-ip>` or `localhost` for USB).

Example:

```text
https://192.168.1.42:8012?ws=wss://192.168.1.42:8012
```

**USB tethered Quest (ADB reverse — use `localhost` on both):**

```text
https://localhost:8012?ws=wss://localhost:8012
```

See [Meta Quest 3 USB (ADB) setup](adb_vr_connect/README.md) for one-time ADB/udev setup and the per-session `adb_vr_connect/connect.sh` script.

### What the dashboard launches

Containers (all must be running):

| Container | Role |
|-----------|------|
| `cyclo_lab` | Isaac Sim recorder / mimic pipeline |
| `robotis-applications` | Vuer VR publisher (`ros2 launch robotis_vuer vr.launch.py`, port **8012**) |
| `ai_worker` | Motion controller (`cyclo_motion_controller_ros`, `controller_type:=vr`) |

Robot-specific launch (picked automatically from task):

| Robot | VR model | Controller | Notes |
|-------|----------|------------|-------|
| **SG2 (gripper)** | `model:=sg2` | `hand:=false` | Lift on right thumbstick |
| **SH5 (hands)** | `model:=sh5` | `hand:=true` | VR lift publishing off; use **I**/**O** in Isaac |

`ROS_DOMAIN_ID=30` (default). Optional: `VR_IMAGE=1` before starting the dashboard enables stereo passthrough background in the headset.

### WiFi connection (every session)

1. Start all three containers (see [Start Containers](#start-containers)).
2. Enable GUI: `xhost +local:docker` (and `xhost +` if Isaac window fails).
3. Start the dashboard: `python3 sg2_ltable_dashboard.py` → open `http://localhost:8765`.
4. Select **Robot** and **Task**.
5. Click **Launch VR + Controller** (teleop only) or **Launch Record** (teleop + Isaac recorder).
6. Wait until dashboard shows `vr: running` and `ai: running`, and the **Vuer** link appears.
7. On the Quest (same WiFi as the PC), open **Meta Quest Browser** and paste the full URL:
   `https://<host-ip>:8012?ws=wss://<host-ip>:8012`
8. Accept the self-signed certificate: **Advanced → Proceed (unsafe)** (may appear twice — page and WebSocket).
9. Click **Enter VR** and allow hand tracking.
10. Confirm the Vuer terminal log in the `robotis-applications` container shows a client connected.

### USB / ADB connection (recommended for recording)

Lower latency jitter than WiFi; no router between Quest and PC.

1. Complete [one-time ADB setup](adb_vr_connect/README.md) (udev rule, USB debugging).
2. Plug Quest into the PC with a **USB 3.0 data cable**.
3. Start Vuer from the dashboard (**Launch VR + Controller** or **Launch Record**).
4. Run the tether script:
   ```bash
   cd ~/AIWORKER/adb_vr_connect
   ./connect.sh
   ```
5. In Meta Quest Browser open:
   `https://localhost:8012?ws=wss://localhost:8012`
6. Accept cert → **Enter VR** → allow hand tracking.

Re-run `./connect.sh` after every USB reconnect (`adb reverse` resets on unplug).

### Enable robot control after connecting

**SG2 (gripper):** Squeeze **both** controller grips to enable VR publishing.

**SH5 (hands):** VR publishing starts **disabled**. Enable with the SH5 hand gesture, or:

```bash
docker exec -it robotis-applications bash
export ROS_DOMAIN_ID=30
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 topic pub --once /vr/reactivate std_msgs/msg/Bool "{data: true}"
```

**SH5 lift:** VR does not publish lift for hand tasks. Adjust from the Isaac Sim window: **I** (up) / **O** (down), range about **−0.40 m to 0.0 m** (reset pose ~−0.25 m).

**SG2 lift:** Right thumbstick on the VR controller.

### Recording demos (Isaac window focus)

| Key | Action |
|-----|--------|
| **B** | Start / stop recording |
| **N** | Save episode (manual mode) |
| **R** | Reset / skip episode |
| **L** | Trigger L-motion (rotate + drive to table) |

L-motion uses kinematic root teleport (swerve off) so carried boxes stay stable. After gripping ~2 s, L-motion can auto-start when `teleop_auto_l_on_grip_s` is set on the task.

When the env `success` termination fires (box placed), the recorder auto-saves if **B** is active and dashboard **Save episode** is set to **Auto on task success**.

Raw datasets are written to `~/AIWORKER/cyclo_lab/datasets/*_raw.hdf5`. The recorder runs inside the `cyclo_lab` container at `/workspace/cyclo_lab` — host edits must be visible there (sync overlay if needed).

### VR troubleshooting

| Symptom | Fix |
|---------|-----|
| Quest cannot reach Vuer | Same WiFi as PC; check firewall on port **8012**; confirm `robotis-applications` container is up |
| Mixed-content / WebSocket error | Use `wss://` in `ws=` when the page is `https://` |
| Certificate warning | **Advanced → Proceed** on Quest browser |
| Robot does not move (SH5) | Enable publishing (gesture or `/vr/reactivate`) |
| Robot does not move (SG2) | Squeeze both grips |
| Stuttery teleop on WiFi | Switch to [USB ADB tethering](adb_vr_connect/README.md) |
| `adb devices` shows `no permission` | Follow udev steps in `adb_vr_connect/README.md` |
| Isaac GUI missing | `xhost +` on host, `DISPLAY=:1` in container |

ROBOTIS upstream reference: [VR teleoperation guide](https://docs.robotis.com/docs/systems/aiworker/quick_start_guide/operation_guide/vr_teleoperation).

## Datasets

Recorded `.hdf5` datasets are intentionally excluded. Re-record demonstrations on the target machine using the dashboard.

SG2 actions stay 19-dimensional (`gripper_l_joint1` only). SH5 actions are 57-dimensional (arms + 20 finger joints per hand + head + lift).

### SG2 Mimic pipeline (after recording)

Each SG2 dashboard task has a matching `Cyclo-Real-Mimic-*` env (see task `mimic_id` in `sg2_ltable_dashboard.py`). You can run the full flow from the dashboard **Mimic pipeline** section, or manually inside the `cyclo_lab` container. Example for L-table:

```bash
# Inside cyclo_lab container
RAW=./datasets/ffw_sg2_l_table_raw.hdf5
IK=./datasets/ffw_sg2_l_table_ik.hdf5

python scripts/sim2real/imitation_learning/mimic/action_data_converter.py \
  --robot_type FFW_SG2 --input_file "$RAW" --output_file "$IK" --action_type ik

python scripts/sim2real/imitation_learning/mimic/annotate_demos.py \
  --task Cyclo-Real-Mimic-Pick-Place-LTable-FFW-SG2-v0 --auto \
  --input_file "$IK" --output_file ./datasets/ffw_sg2_l_table_annotate.hdf5 --enable_cameras --headless

python scripts/sim2real/imitation_learning/mimic/generate_dataset.py \
  --device cuda --num_envs 10 --task Cyclo-Real-Mimic-Pick-Place-LTable-FFW-SG2-v0 \
  --generation_num_trials 500 --input_file ./datasets/ffw_sg2_l_table_annotate.hdf5 \
  --output_file ./datasets/ffw_sg2_l_table_generate.hdf5 --enable_cameras --headless

python scripts/sim2real/imitation_learning/mimic/action_data_converter.py \
  --robot_type FFW_SG2 --input_file ./datasets/ffw_sg2_l_table_generate.hdf5 \
  --output_file ./datasets/ffw_sg2_l_table_joint.hdf5 --action_type joint
```

Basket pick-place uses `Cyclo-Real-Mimic-Pick-Place-FFW-SG2-v0` (upstream env, not in overlay).

### SH5 Mimic pipeline (after recording)

Same steps as SG2 (dashboard pipeline or manual commands). Use `--robot_type FFW_SH5` and the SH5 `mimic_id` from the dashboard (e.g. `Cyclo-Real-Mimic-Pick-Place-LTable-FFW-SH5-v0`). IK actions are 57-dim: both arm EEF poses plus all finger/head/lift joints. During datagen, finger poses are taken from `joint_pos_target` in source demos (not the 1D curl proxy in the Mimic API).

```bash
RAW=./datasets/ffw_sh5_l_table_raw.hdf5
IK=./datasets/ffw_sh5_l_table_ik.hdf5

python scripts/sim2real/imitation_learning/mimic/action_data_converter.py \
  --robot_type FFW_SH5 --input_file "$RAW" --output_file "$IK" --action_type ik

python scripts/sim2real/imitation_learning/mimic/annotate_demos.py \
  --task Cyclo-Real-Mimic-Pick-Place-LTable-FFW-SH5-v0 --auto \
  --input_file "$IK" --output_file ./datasets/ffw_sh5_l_table_annotate.hdf5 --enable_cameras --headless

python scripts/sim2real/imitation_learning/mimic/generate_dataset.py \
  --device cuda --num_envs 10 --task Cyclo-Real-Mimic-Pick-Place-LTable-FFW-SH5-v0 \
  --generation_num_trials 500 --input_file ./datasets/ffw_sh5_l_table_annotate.hdf5 \
  --output_file ./datasets/ffw_sh5_l_table_generate.hdf5 --enable_cameras --headless

python scripts/sim2real/imitation_learning/mimic/action_data_converter.py \
  --robot_type FFW_SH5 --input_file ./datasets/ffw_sh5_l_table_generate.hdf5 \
  --output_file ./datasets/ffw_sh5_l_table_joint.hdf5 --action_type joint

lerobot-python scripts/sim2real/imitation_learning/data_converter/isaaclab2lerobot.py \
  --task=Cyclo-Real-Pick-Place-LTable-FFW-SH5-v0 \
  --robot_type FFW_SH5 \
  --dataset_file ./datasets/ffw_sh5_l_table_joint.hdf5
```

SG2 LeRobot export (after joint convert):

```bash
lerobot-python scripts/sim2real/imitation_learning/data_converter/isaaclab2lerobot.py \
  --task=Cyclo-Real-Pick-Place-LTable-FFW-SG2-v0 \
  --robot_type FFW_SG2 \
  --dataset_file ./datasets/ffw_sg2_l_table_joint.hdf5
```

### Sim inference (VR + DDS SDK)

Run a task env with the teleop SDK in **inference** mode (`B` enable control, `R` reset, `L` L-motion; SH5 lift via **I**/**O**):

```bash
python scripts/sim2real/imitation_learning/inference/inference_demos.py \
  --task Cyclo-Real-Pick-Place-LTable-FFW-SH5-v0 \
  --robot_type FFW_SH5 --enable_cameras
```

SG2: `--robot_type FFW_SG2` and the matching `Cyclo-Real-*-FFW-SG2-v0` task id.

### Robomimic play (L-table, SG2)

For L-table policy evaluation, use robomimic play with SG2 action remap and optional scripted base L-motion:

```bash
python scripts/imitation_learning/robomimic/play.py \
  --device cuda \
  --task Cyclo-Real-Pick-Place-LTable-FFW-SG2-v0 \
  --checkpoint /PATH/TO/model_epoch_20.pth \
  --num_rollouts 10 --horizon 2000 \
  --enable_cameras --action_mode inference \
  --scripted_l_motion
```

Flags:
- `--action_mode inference`: initializes SG2 real-task action terms
- `--remap_ffw_sg2_actions`: swaps head/lift action indices for SG2 19-dim joint datasets
- `--scripted_l_motion`: runs base rotate+forward L-motion in play after grasp latch

## Overlay Sync

`overlays/cyclo_lab/` is the source of truth for AIWORKER changes.

Push overlay onto a live checkout:

```bash
./sync_overlay.sh ~/path/to/cyclo_lab
```

Pull edits made directly in `cyclo_lab` back into the overlay:

```bash
./pull_overlay.sh ~/path/to/cyclo_lab
```

Fresh installs use `./setup.sh`, which clones upstream pins and rsyncs the overlay automatically. Upstream commit hashes in `manifest.json` are bootstrap pins; teleop fixes ship in the overlay.

## Notes

- This overlay assumes NVIDIA Docker and Isaac Sim container requirements are already satisfied.
- `cyclo_lab/docker/.env` is included to pin Isaac Sim 5.1 settings used during development.
- If you change upstream commits, re-test the overlay because task registration and IsaacLab APIs may move.
