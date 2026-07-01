# AIWORKER

Portable overlay for AI Worker Isaac Sim VR teleoperation tasks.

This repo is intentionally small. It does not vendor the full ROBOTIS repositories and it does not include recorded datasets. Instead, `setup.sh` clones the required upstream repos at pinned commits, initializes `cyclo_lab` submodules, and copies the AIWORKER overlay files on top.

## What This Adds

- `cyclo_lab` dashboard for launching the stack, selecting tasks, and running the Mimic pipeline step-by-step.
- SG2 basket pick-place, L-table, box-stack, single-box-far, and thick-box variants (record + Mimic for SG2).
- SH5 hand versions of the same tasks (record + Mimic).
- SH5 DDS recorder support for VR hand teleoperation.
- Task-specific table/box assets and teleop motion settings.
- Minor `robotis_applications` VR publisher update used by this setup.

## Upstream Pins

- `cyclo_lab`: `85b237bf22068da18999bacbda5652f201594d11`
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

## VR Notes

For SH5 hand tasks, the dashboard launches:

- `robotis_vuer vr.launch.py model:=sh5`
- `cyclo_motion_controller_ros ai_worker_controller.launch.py controller_type:=vr hand:=true`

VR lift publishing is disabled for hand tasks. Adjust lift from the Isaac Sim window with **I** (up) and **O** (down); range is about **−0.40 m to 0.0 m** (reset pose starts around −0.25 m).

L-motion on both SG2 and SH5 uses kinematic root teleport (swerve wheels off) so carried boxes stay stable during recording. After gripping a box for ~2 s, L-motion can auto-start when `teleop_auto_l_on_grip_s` is set on the task.

Recording controls (Isaac window focus): **B** start, **L** L-motion, **N** save episode, **R** reset/skip. When the env `success` termination fires (box placed), the recorder also auto-saves if **B** is active.

The dashboard recorder runs inside the Isaac container at `/workspace/cyclo_lab`. Ensure that path mounts your synced `cyclo_lab` checkout (host edits must be visible there).

```text
https://<host-ip>:8012
```

The headset page is printed by the dashboard. It normally looks like the URL above.

For hand tasks, VR publishing starts disabled by default. Enable it with the SH5 gesture or publish the override:

```bash
docker exec -it robotis-applications bash
export ROS_DOMAIN_ID=30
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 topic pub --once /vr/reactivate std_msgs/msg/Bool "{data: true}"
```

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
