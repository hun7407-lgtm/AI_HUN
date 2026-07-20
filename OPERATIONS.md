# OPERATIONS — 수동 커맨드 레퍼런스

**작성자:** Hun Kim (hun7728@hanyang.ac.kr) · **수정일:** 2026-07-20

대시보드 대신 컨테이너 내부에서 직접 실행할 때 쓰는 상세 커맨드. 모든 명령은 `cyclo_lab` 컨테이너 안, `/workspace/cyclo_lab`에서 실행합니다.

```bash
docker exec -it cyclo_lab bash -lc 'cd /workspace/cyclo_lab && ...'
```

---

## 1. SG2 Mimic 파이프라인 (수동)

```bash
RAW=./datasets/ffw_sg2_l_table_raw.hdf5
IK=./datasets/ffw_sg2_l_table_ik.hdf5

# 1) IK 변환
python scripts/sim2real/imitation_learning/mimic/action_data_converter.py \
  --robot_type FFW_SG2 --input_file "$RAW" --output_file "$IK" --action_type ik

# 2) Annotate
python scripts/sim2real/imitation_learning/mimic/annotate_demos.py \
  --task Cyclo-Real-Mimic-Pick-Place-LTable-FFW-SG2-v0 --auto \
  --input_file "$IK" --output_file ./datasets/ffw_sg2_l_table_annotate.hdf5 \
  --enable_cameras --headless

# 3) Datagen (500개 증강)
python scripts/sim2real/imitation_learning/mimic/generate_dataset.py \
  --device cuda --num_envs 10 --task Cyclo-Real-Mimic-Pick-Place-LTable-FFW-SG2-v0 \
  --generation_num_trials 500 --input_file ./datasets/ffw_sg2_l_table_annotate.hdf5 \
  --output_file ./datasets/ffw_sg2_l_table_generate.hdf5 --enable_cameras --headless

# 4) Joint 변환 (robomimic 학습용)
python scripts/sim2real/imitation_learning/mimic/action_data_converter.py \
  --robot_type FFW_SG2 --input_file ./datasets/ffw_sg2_l_table_generate.hdf5 \
  --output_file ./datasets/ffw_sg2_l_table_joint.hdf5 --action_type joint

# 5) (선택) LeRobot export
lerobot-python scripts/sim2real/imitation_learning/data_converter/isaaclab2lerobot.py \
  --task=Cyclo-Real-Pick-Place-LTable-FFW-SG2-v0 \
  --robot_type FFW_SG2 --dataset_file ./datasets/ffw_sg2_l_table_joint.hdf5
```

> **주의(visualization)**: datagen을 시각화로 보려면 `--headless`를 빼고 `--num_envs`를 2~6으로 낮춰 `-it`로 실행. 헤드리스 대량 생성이 훨씬 빠름.

---

## 2. SH5 Mimic 파이프라인 (수동)

SG2와 동일한 단계에 `--robot_type FFW_SH5`, SH5 `mimic_id` 사용. IK 액션은 57차원(양팔 EEF + 손가락/헤드/리프트). datagen 시 손가락 포즈는 소스 데모의 `joint_pos_target`에서 가져옴.

```bash
RAW=./datasets/ffw_sh5_l_table_raw.hdf5
IK=./datasets/ffw_sh5_l_table_ik.hdf5

python scripts/sim2real/imitation_learning/mimic/action_data_converter.py \
  --robot_type FFW_SH5 --input_file "$RAW" --output_file "$IK" --action_type ik

python scripts/sim2real/imitation_learning/mimic/annotate_demos.py \
  --task Cyclo-Real-Mimic-Pick-Place-LTable-FFW-SH5-v0 --auto \
  --input_file "$IK" --output_file ./datasets/ffw_sh5_l_table_annotate.hdf5 \
  --enable_cameras --headless

python scripts/sim2real/imitation_learning/mimic/generate_dataset.py \
  --device cuda --num_envs 10 --task Cyclo-Real-Mimic-Pick-Place-LTable-FFW-SH5-v0 \
  --generation_num_trials 500 --input_file ./datasets/ffw_sh5_l_table_annotate.hdf5 \
  --output_file ./datasets/ffw_sh5_l_table_generate.hdf5 --enable_cameras --headless

python scripts/sim2real/imitation_learning/mimic/action_data_converter.py \
  --robot_type FFW_SH5 --input_file ./datasets/ffw_sh5_l_table_generate.hdf5 \
  --output_file ./datasets/ffw_sh5_l_table_joint.hdf5 --action_type joint

lerobot-python scripts/sim2real/imitation_learning/data_converter/isaaclab2lerobot.py \
  --task=Cyclo-Real-Pick-Place-LTable-FFW-SH5-v0 \
  --robot_type FFW_SH5 --dataset_file ./datasets/ffw_sh5_l_table_joint.hdf5
```

---

## 3. robomimic 학습 (Train)

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

---

## 4. 체크포인트 Play (L-table, SG2)

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

플래그:
- `--action_mode inference` : SG2 real-task 액션 텀 초기화
- `--remap_ffw_sg2_actions` : SG2 19차원 관절 데이터셋의 head/lift 인덱스 스왑
- `--scripted_l_motion` : grasp latch 후 base 회전+전진 L-motion 수행

---

## 5. Sim 추론 (VR + DDS SDK, inference 모드)

`B` 제어 활성 / `R` 리셋 / `L` L-motion (SH5 리프트는 **I**/**O**):

```bash
python scripts/sim2real/imitation_learning/inference/inference_demos.py \
  --task Cyclo-Real-Pick-Place-LTable-FFW-SH5-v0 \
  --robot_type FFW_SH5 --enable_cameras
# SG2: --robot_type FFW_SG2 및 매칭 Cyclo-Real-*-FFW-SG2-v0 task id
```

---

## 6. 데이터 검증 (replay)

녹화/생성 HDF5가 온전한지 특정 에피소드를 재생해 확인:

```bash
docker exec -e DISPLAY=:1 -e TERM=xterm -it cyclo_lab bash -lc '
cd /workspace/cyclo_lab
./third_party/IsaacLab/isaaclab.sh -p scripts/tools/replay_demos.py \
  --task Cyclo-Real-Mimic-Pick-Place-LTable-FFW-SG2-v0 \
  --dataset_file ./datasets/ffw_sg2_l_table_generate.hdf5 \
  --select_episodes 0 100 250 400 499 --enable_cameras
'
```
