# 데이터 수집 파이프라인 — FFW-SG2 VR 원격조작

**작성자:** Hun Kim (hun7728@hanyang.ac.kr) · **수정일:** 2026-07-20

NVIDIA Isaac Sim에서 Meta Quest VR 원격조작으로 양팔 매니퓰레이션 **+ 모바일 베이스** 시연을
수집하고, ROBOTIS IsaacLab-Mimic 증강 파이프라인을 돌린 뒤, 모방학습 정책 학습에 쓰는
**LeRobot** 포맷으로 내보내는 전 과정 가이드.

이 문서는 실제 파이프라인이 진행되는 순서 — **셋업 → 환경 → VR 연결 → 원격조작 → 녹화 →
증강 → 변환** — 대로 정렬되어 있으며, stock(업스트림) 동작과 이 fork에서 변경한 부분을 모두
설명한다. 업스트림 위에 추가한 작업은 **[추가]** / **[수정]** 으로 표시했고, 건드린 파일은
전부 [부록 A](#부록-a--추가--수정-파일)에 정리했다.

**레포 계보.** 이 레포(`AI_HUN`, https://github.com/hun7407-lgtm/AI_HUN)는 베이스 레포
**`EKAIWORKER`**(https://github.com/Disniekie01/EKAIWORKER) 위에 얹은 오버레이다. EKAIWORKER는
다시 ROBOTIS 업스트림 3개(`cyclo_lab`, `ai_worker`, `robotis_applications`)를 고정 커밋으로
핀한다. 여기 `overlays/` 아래의 모든 것은 `setup.sh`가 그 업스트림 위에 적용한다.

---

## 목차

1. [개요 & 시스템 구조](#1-개요--시스템-구조)
2. [셋업](#2-셋업)
3. [시뮬레이션 환경 & 태스크](#3-시뮬레이션-환경--태스크)
4. [VR 원격조작 시스템](#4-vr-원격조작-시스템)
5. [원격조작 조작법](#5-원격조작-조작법)
6. [시연 녹화](#6-시연-녹화)
7. [IsaacLab-Mimic 증강 파이프라인](#7-isaaclab-mimic-증강-파이프라인)
8. [LeRobot 변환 & 출력 스키마](#8-lerobot-변환--출력-스키마)
9. [실물 정합](#9-실물-정합)
10. [주행 가능 모바일 베이스](#10-주행-가능-모바일-베이스)
11. [알려진 제약](#11-알려진-제약)
12. [부록 A — 추가 / 수정 파일](#부록-a--추가--수정-파일)

---

## 1. 개요 & 시스템 구조

### 1.1 목표

시뮬레이션에서 **실물 `ffw_sg2_rev1` 로봇과 동일한 포맷**으로 시연을 수집 → sim/real 시연을
함께 모아 같은 정책을 학습. 시연 1개는 각 시점마다 관절 상태, 베이스 속도, 명령(action),
RGB 카메라 4개를 담는다.

### 1.2 로봇

ROBOTIS **FFW-SG2** — 양팔 모바일 매니퓰레이터:

| 서브시스템 | DoF | 비고 |
|---|---|---|
| 좌팔 | 7 | `arm_l_joint1..7` |
| 좌 그리퍼 | 구동 1 (+mimic 3) | `gripper_l_joint1` 구동; `2..4`는 PhysX mimic 링키지로 추종 |
| 우팔 | 7 | `arm_r_joint1..7` |
| 우 그리퍼 | 구동 1 (+mimic 3) | `gripper_r_joint1` |
| 헤드 | 2 | `head_joint1`(pitch), `head_joint2`(yaw) |
| 리프트 | 1 | `lift_joint` (수직 몸통 리프트, prismatic) |
| 베이스 | 스워브 3모듈 | 좌/우/후, 각각 steer + drive |

**action / 관절 상태는 19차원**(7+1 + 7+1 + 2 + 1). mobile 태스크에선 **22차원**(+ 베이스
`linear_x, linear_y, angular_z`).

### 1.3 태스크

**L-테이블 pick & place:** 앞 테이블의 박스를 양 그리퍼로 집어 옮겨 왼쪽("L") 테이블에 놓기.
집는 위치와 놓는 위치 사이에서 베이스가 이동해야 한다.

### 1.4 컨테이너 3개

데이터 수집은 ROS 2(`ROS_DOMAIN_ID=30`, RMW = `rmw_fastrtps_cpp` = Fast DDS)로 통신하는 Docker
컨테이너 3개에 걸쳐 돌아간다:

| 컨테이너 | 이미지 | 역할 | 포트 |
|---|---|---|---|
| `cyclo_lab` | `cyclolab/cyclo-lab:latest` (Isaac Sim 5.1.0 base) | Isaac Sim, 태스크 env, recorder, Mimic 파이프라인, **대시보드** | 8765 |
| `robotis-applications` | `robotis/robotis-applications:1.0.0` | **Vuer VR 퍼블리셔** (`vr_publisher_sg2`) | 8012 |
| `ai_worker` | `robotis/ai-worker:2.0.0` | **팔 IK 컨트롤러** (`vr_controller`) | — |

### 1.5 전체 데이터 흐름

```
[작업자 + Meta Quest 3]
   │  손/컨트롤러 포즈 + 조이스틱  (WebXR)
   ▼
[robotis-applications]  Vuer VR 퍼블리셔
   ├── 팔/손목 포즈 ─(DDS)─►  [ai_worker] vr_controller ── IK ── 관절 명령 ─(DDS)─┐
   └── /cmd_vel (Twist) ──────────────────────────────────────────────────────┐ │
                                                                              ▼ ▼
[cyclo_lab]  Isaac Sim + record_demos.py + FFWSG2Sdk
   │   • 팔 관절 명령을 sim 로봇에 적용
   │   • /cmd_vel로 스워브 베이스 구동  (mobile 태스크)
   │   • 카메라 4개 렌더 + 관절/베이스 상태 읽기
   ▼
  raw .hdf5
   │
   ├─► IK 변환 ─► annotate ─► IsaacLab-Mimic datagen ─► joint 변환 ─► LeRobot 내보내기
   ▼
LeRobot 데이터셋  (카메라 4개 + 22차원 state/action)
```

---

## 2. 셋업

### 2.1 사전 준비 (`setup.sh`가 설치 안 함)

| 요구사항 | 확인 |
|---|---|
| Linux 호스트 (Ubuntu / Pop!_OS) + X11 | — |
| NVIDIA RTX GPU + 드라이버 (검증: RTX PRO 5000, 24 GB) | `nvidia-smi` |
| Docker Engine + NVIDIA Container Toolkit | `docker run --gpus all nvidia/cuda:12.4.0-base-ubuntu22.04 nvidia-smi` |
| NGC 로그인 (`nvcr.io/nvidia/isaac-sim:5.1.0` pull) | `docker login nvcr.io` |
| Meta Quest 3 (+ USB 테더링용 최초 1회 ADB/udev) | `adb_vr_connect/README.md` |
| 디스크 (이미지 + 데이터셋) | 수백 GB |

### 2.2 클론 & 빌드

```bash
git clone https://github.com/hun7407-lgtm/AI_HUN.git AIWORKER
cd AIWORKER
./setup.sh ~/AIWORKER
```

`setup.sh` 동작 순서:
1. ROBOTIS 업스트림 3개를 **고정 커밋**으로 `git clone`
   (`cyclo_lab`, `ai_worker`, `robotis_applications`).
2. `cyclo_lab` 서브모듈 초기화(`git submodule update --init --recursive`) → Isaac Lab을
   `cyclo_lab/third_party/IsaacLab`로 가져옴.
3. fork의 `overlays/cyclo_lab/`, `overlays/robotis_applications/`를 위에 `rsync`.

결과: `~/AIWORKER/{cyclo_lab, ai_worker, robotis_applications}`. 이후 오버레이 수정본을 라이브에
다시 반영하려면 `./sync_overlay.sh ~/AIWORKER/cyclo_lab`(오버레이 → 라이브), 반대로는
`./pull_overlay.sh`(라이브 → 오버레이).

### 2.3 컨테이너 시작 & GUI 허용

```bash
cd ~/AIWORKER/cyclo_lab/docker            && ./container.sh start
cd ~/AIWORKER/robotis_applications/docker && ./container.sh start
cd ~/AIWORKER/ai_worker/docker            && ./container.sh start

# 호스트: 컨테이너가 로컬 X 서버에 GUI 창을 열도록 허용
xhost +local:docker
xhost +SI:localuser:root      # (Isaac 창이 안 뜨면 `xhost +`)
```

---

## 3. 시뮬레이션 환경 & 태스크

### 3.1 Isaac Sim + Isaac Lab

- **Isaac Sim 5.1.0** — 물리 + 렌더 엔진 (베이스 컨테이너 이미지).
- **Isaac Lab 2.3.0** — 태스크 프레임워크: 씬 정의, 매니저 시스템(관측/액션/이벤트/종료/
  recorder), IsaacLab-Mimic 데이터셋 도구. 서브패키지: `isaaclab 0.47.2`,
  `isaaclab_mimic 1.0.15`, `isaaclab_tasks 0.11.6`, `isaaclab_rl 0.4.4`,
  `isaaclab_assets 0.2.3`.

### 3.2 매니저 기반 RL env로서의 L-테이블 태스크

태스크는 Isaac Lab `ManagerBasedRLEnvCfg`로 정의된다(`pick_place_env_cfg.py` +
`joint_pos_env_cfg.py`). 데이터 수집 관련 매니저:

**Scene** — 로봇, 앞/왼쪽 테이블, 박스, 박스 riser, drop-zone 마커, 바닥/조명, 카메라.

**Actions (총 19차원)** — 6개 액션 항:

| 항 | 조인트 | dim |
|---|---|---|
| `arm_l_action` | `arm_l_joint1..7` | 7 |
| `gripper_l_action` | `gripper_l_joint1` | 1 |
| `arm_r_action` | `arm_r_joint1..7` | 7 |
| `gripper_r_action` | `gripper_r_joint1` | 1 |
| `head_action` | `head_joint1..2` | 2 |
| `lift_action` | `lift_joint` | 1 |

**Observations (`policy` 그룹)** — 매 시점 기록:

| 항 | 내용 |
|---|---|
| `actions` | 직전 액션 |
| `joint_pos` | 관절 위치 19개 |
| `joint_pos_target` | 관절 타겟 19개 |
| `left_eef_pose` / `right_eef_pose` | 엔드이펙터 포즈 (frame transformer) |
| `cam_head`, `cam_right_head`, `cam_left_wrist`, `cam_right_wrist` | RGB 이미지 **[수정 — §9]** |
| `base_velocity` | `[linear_x, linear_y, angular_z]` (mobile 태스크만) **[추가]** |

**Recorder** — `ActionStateRecorderManagerCfg`가 각 에피소드의 액션 + 관측 + 초기 상태를 HDF5
그룹 `data/demo_<i>`에 기록.

**Terminations** — `time_out`, task `success`(박스가 왼쪽 테이블 위), `object_dropped`.

### 3.3 태스크 변형

| 태스크 ID | 베이스 | 베이스 이동 | 차원 | 용도 |
|---|---|---|---|---|
| `Cyclo-Real-Pick-Place-LTable-FFW-SG2-v0` | 용접 | scripted L-motion (키네마틱 텔레포트) | 19 | stock (팔만) |
| `Cyclo-Real-Mimic-Pick-Place-LTable-FFW-SG2-v0` | 용접 | (datagen이 사용) | 19 | Mimic 증강 env |
| `Cyclo-Real-Pick-Place-LTable-Mobile-FFW-SG2-v0` **[추가]** | 자유 | 작업자가 `/cmd_vel`로 주행 | 22 | 모바일 베이스 수집 |

---

## 4. VR 원격조작 시스템

### 4.1 Vuer / WebXR

작업자는 Meta Quest 3를 쓰고 Quest 브라우저에서 **Vuer**(WebXR 페이지)를 연다. Vuer는 헤드셋/
컨트롤러 포즈와 버튼 상태를 WebSocket으로 PC에 스트리밍한다(앱 설치 불필요). `robotis_vuer`
노드(`vr_publisher_sg2`)가 포트 **8012**에서 Vuer를 서빙한다.

### 4.2 퍼블리셔 → 컨트롤러 → sim 체인

1. **`vr_publisher_sg2`**(`robotis-applications`)가 WebXR 스트림을 받아 ROS 2 / DDS로 발행:
   손목/팔꿈치/어깨 포즈, 그리퍼 squeeze, 리프트/헤드 조이스틱 명령, 그리고 베이스용
   **`/cmd_vel`**(`Twist`).
2. **`vr_controller`**(`ai_worker`)가 손목 포즈를 구독해 역기구학(IK)을 풀고 팔 관절 궤적을
   발행.
3. **`FFWSG2Sdk`**(`cyclo_lab`, `dds_sdk/ffw_sg2_sdk.py`)가 팔/리프트/헤드 궤적을 구독해 sim
   로봇에 적용; **`/cmd_vel`**도 구독해 스워브 베이스를 구동(mobile 태스크) **[수정 — §10]**.

셋 다 `ROS_DOMAIN_ID=30`. 명령 토픽(팔, 리프트, 헤드, `/cmd_vel`)은 **RELIABLE** QoS를 써서
DDS가 SDK의 reliable 리더에 확실히 전달한다.

### 4.3 Quest 연결

**USB / ADB 테더 (권장 — 지터 최소):**
```bash
cd ~/AIWORKER/adb_vr_connect && ./connect.sh     # 케이블 재연결 때마다 다시 실행
# Quest 브라우저:  https://localhost:8012?ws=wss://localhost:8012  → 인증서 수락 → Enter VR
```
최초 1회 ADB/udev 세팅: `adb_vr_connect/README.md`.

**WiFi (Quest 브라우저 2단계):**
1. 인증서 수락: `https://<PC-IP>:8012` → Advanced → Proceed.
2. 새 탭: `https://vuer.ai/?ws=wss://<PC-IP>:8012` → Enter VR → 핸드트래킹 허용.

---

## 5. 원격조작 조작법

| 입력 | 동작 |
|---|---|
| **양쪽 그립 ~3초 홀드** | 팔 teleop(및 베이스 cmd_vel) 활성 |
| VR에서 손 이동 | 팔 엔드이펙터가 따라감(IK); squeeze로 그리퍼 닫힘 |
| 오른쪽 썸스틱 X | 리프트 상승/하강 |
| **왼쪽 Y 버튼** | 베이스 모드 토글: `LIFT+HEAD` ↔ `LIFT+CMD_VEL` **[추가]** |
| 왼쪽 썸스틱 (`CMD_VEL` 모드) | 베이스 이동 (전후, 게걸음) |
| **A 버튼 (홀드)** | 베이스 우회전 (`angular.z < 0`) — 두 모드 다 **[추가]** |
| **B 버튼 (홀드)** | 베이스 좌회전 (`angular.z > 0`) — 두 모드 다 **[추가]** |
| 왼쪽 썸스틱 (`LIFT+HEAD` 모드) | 헤드 pan/tilt |

참고: A + B 동시는 상쇄됨(하나씩). Vuer 매핑상 각 컨트롤러는 `aButton`/`bButton`을 노출하므로
물리 **Y**는 왼쪽 컨트롤러의 `bButton`이다. 시작/방향전환 시 짧은 지연은 정상적인 스워브
동작이다(바퀴가 재조향 후 베이스 이동).

---

## 6. 시연 녹화

### 6.1 스택 실행

```bash
cd ~/AIWORKER/cyclo_lab
python3 sg2_ltable_dashboard.py            # http://localhost:8765
```

대시보드에서:
1. 태스크 선택 — 예: **"L-Table Pick & Place (mobile base)"** **[대시보드에 추가됨]**.
2. **Launch Record** — Isaac Sim(recorder 포함) + VR 퍼블리셔 + 컨트롤러 실행.
   `vr: running`, `ai: running` 뜰 때까지 대기.
3. Quest 연결(§4.3) 후 Enter VR.

### 6.2 take별 녹화 사이클 (Isaac 창 포커스)

| 키 | 동작 |
|---|---|
| **B** | 이 take 녹화 시작 + 팔 teleop (매 take 전에) |
| **N** | 에피소드 저장 |
| **R** | take 폐기 후 재시작 |

mobile 태스크에선 매니퓰레이션하면서 조이스틱(§5)으로 베이스를 몬다. 성공(박스 놓음) 시
설정에 따라 자동 저장 가능.

### 6.3 무엇이, 어디에 기록되나

저장된 각 에피소드는 HDF5 그룹이 된다:

```
data/demo_<i>/
  initial_state/…                 # reset 시점의 아티큘레이션 + rigid-object 포즈
  actions            (T, 19|22)   # 스텝별 액션
  obs/joint_pos      (T, 19)      # 관절 위치
  obs/joint_pos_target (T, 19)
  obs/base_velocity  (T, 3)       # mobile 태스크만  [추가]
  obs/cam_head       (T, 376, 672, 3)   uint8 HWC     [수정]
  obs/cam_right_head (T, 376, 672, 3)                 [추가]
  obs/cam_left_wrist (T, 424, 240, 3)                 [추가]
  obs/cam_right_wrist(T, 424, 240, 3)                 [추가]
  attrs: success, seed, num_samples
```

**세션 기반 데이터셋 네이밍 [수정]** — 대시보드는 이렇게 저장:
```
datasets/{base}/{YYYYMMDD}/{HHMM}_{ip}_{src}_{base}_{stage}.hdf5
#  ip = 호스트 IPv4 마지막 옥텟   src = vr|leader   stage = raw|ik|annotate|gen|joint
```
대시보드에서 `USE_SESSION_NAMING = False`면 기존 flat `datasets/<base>_<stage>.hdf5`로 복귀.

녹화의 `raw` 파일이 증강 파이프라인(§7)의 입력이다.

---

## 7. IsaacLab-Mimic 증강 파이프라인

소수의 원격조작 시연을 **IsaacLab-Mimic**으로 수백 개의 학습 시연으로 증강한다. 대시보드
**Mimic pipeline** 섹션에서 한 단계씩, 또는 `cyclo_lab` 컨테이너 안에서 수동으로 실행. 5단계
(SG2 L-테이블 예):

| # | 단계 | 스크립트 | 입력 → 출력 | 하는 일 |
|---|---|---|---|---|
| 1 | **IK 변환** | `action_data_converter.py --action_type ik` | `raw → ik` | 녹화된 **관절** 액션을 Mimic이 재조합하는 **엔드이펙터(IK)** 액션으로 변환. |
| 2 | **Annotate** | `annotate_demos.py --auto` | `ik → annotate` | 각 데모의 **서브태스크 구간**(grasp / move / place)을 라벨링 → Mimic이 어디서 이어붙일지 앎. |
| 3 | **Datagen** | `generate_dataset.py` | `annotate → generate` | IsaacLab-Mimic이 새 데모를 **합성**: 라벨된 서브태스크를 박스/테이블/조명 랜덤화 하에 재생·재조합, `--num_envs` 병렬 Isaac 환경에서 `--generation_num_trials` 성공까지 생성(SG2 L-테이블은 리프트/헤드 + 베이스 모션에 `cyclo_mimic_datagen.py` 사용). |
| 4 | **Joint 변환** | `action_data_converter.py --action_type joint` | `generate → joint` | 생성된 IK 액션을 학습용 **관절** 액션으로 되변환. |
| 5 | **LeRobot 내보내기** | `isaaclab2lerobot.py` | `joint → lerobot` | LeRobot 데이터셋으로 내보냄(§8). |

각 단계는 **하나씩** 완료 후 다음으로. 수동 커맨드(`cyclo_lab` 컨테이너 안,
`/workspace/cyclo_lab`):

```bash
RAW=./datasets/ffw_sg2_l_table_raw.hdf5
IK=./datasets/ffw_sg2_l_table_ik.hdf5

# 1) IK 변환
python scripts/sim2real/imitation_learning/mimic/action_data_converter.py \
  --robot_type FFW_SG2 --input_file "$RAW" --output_file "$IK" --action_type ik

# 2) Annotate
python scripts/sim2real/imitation_learning/mimic/annotate_demos.py \
  --task Cyclo-Real-Mimic-Pick-Place-LTable-FFW-SG2-v0 --auto \
  --input_file "$IK" --output_file ./datasets/ffw_sg2_l_table_annotate.hdf5 --enable_cameras --headless

# 3) Datagen (합성 500개, 병렬 10개 env)
python scripts/sim2real/imitation_learning/mimic/generate_dataset.py \
  --device cuda --num_envs 10 --task Cyclo-Real-Mimic-Pick-Place-LTable-FFW-SG2-v0 \
  --generation_num_trials 500 --input_file ./datasets/ffw_sg2_l_table_annotate.hdf5 \
  --output_file ./datasets/ffw_sg2_l_table_generate.hdf5 --enable_cameras --headless

# 4) Joint 변환
python scripts/sim2real/imitation_learning/mimic/action_data_converter.py \
  --robot_type FFW_SG2 --input_file ./datasets/ffw_sg2_l_table_generate.hdf5 \
  --output_file ./datasets/ffw_sg2_l_table_joint.hdf5 --action_type joint
```

튜닝 env 변수(대시보드 시작 전): `GENERATION_NUM_TRIALS`(기본 500),
`PIPELINE_NUM_ENVS`(기본 10). 단계별 로그: `docker exec cyclo_lab tail -f
/tmp/sg2_ltable_pipe_generate.log`.

> 참고: Mimic 경로는 현재 **19차원**(팔만) 데이터 대상이다. mobile 22차원 데이터는 녹화·
> LeRobot 내보내기는 되지만, IK/annotate/datagen이 아직 베이스 속도 차원용으로 배선되지
> 않았다(§11 참고).

---

## 8. LeRobot 변환 & 출력 스키마

### 8.1 변환기 **[수정]**

`isaaclab2lerobot.py`가 녹화 내용을 자동 감지해 실물 스키마로 내보낸다 — 별도 플래그 불필요:

```bash
lerobot-python scripts/sim2real/imitation_learning/data_converter/isaaclab2lerobot.py \
  --task Cyclo-Real-Pick-Place-LTable-Mobile-FFW-SG2-v0 \
  --robot_type FFW_SG2 --fps 15 \
  --dataset_file ./datasets/<...>_joint.hdf5
```

- **카메라** — 녹화된 카메라 스트림을 감지해 각각 `observation.images.rgb.<name>`,
  **채널 우선 `[3, H, W]`**(실물 일치)로 내보냄. `cam_head` → `cam_left_head`; 해상도는 녹화
  배열에서 읽음.
- **베이스 속도** — `obs/base_velocity`가 있으면 `[linear_x, linear_y, angular_z]`를
  `observation.state`와 `action`에 붙여 **22차원**; 없으면 **19차원**.
- 감지된 카메라와 `Base velocity: present -> 22-dim state/action`을 로그로 찍음.

### 8.2 출력 스키마 (실물 `ffw_sg2_rev1`과 일치)

| Feature | Shape | 비고 |
|---|---|---|
| `observation.state` | (22,) | 관절 19 + `linear_x, linear_y, angular_z` |
| `action` | (22,) | 동일 레이아웃 |
| `observation.images.rgb.cam_left_head` | [3, 376, 672] | video, CHW |
| `observation.images.rgb.cam_right_head` | [3, 376, 672] | video, CHW |
| `observation.images.rgb.cam_left_wrist` | [3, 424, 240] | video, CHW |
| `observation.images.rgb.cam_right_wrist` | [3, 424, 240] | video, CHW |

관절 19개 순서: `arm_l_joint1..7, gripper_l_joint1, arm_r_joint1..7, gripper_r_joint1,
head_joint1, head_joint2, lift_joint`.

---

## 9. 실물 정합

실물 `ffw_sg2_rev1`은 RGB 카메라 **4개**와 **22차원** state를 기록하는데, stock sim은 카메라
1개, 19차원이었다. 추가 2가지로 이 갭을 메운다.

### 9.1 카메라 **[수정]**

| LeRobot 키 | Sim 카메라 | 마운트 | 해상도 |
|---|---|---|---|
| `…rgb.cam_left_head` | `cam_head` (ZED 좌안) | `head_link2/zed` | 376 × 672 |
| `…rgb.cam_right_head` | `cam_right_head` | `head_link2/zed`, 대칭 | 376 × 672 |
| `…rgb.cam_left_wrist` | `cam_left_wrist` | `arm_l_link7` (D405 위치) | 424 × 240 |
| `…rgb.cam_right_wrist` | `cam_right_wrist` | `arm_r_link7` (D405 위치) | 424 × 240 |

- 헤드 카메라는 헤드 ZED의 좌/우 눈, `zed` prim 중심 기준 Y 대칭(±0.03 m ⇒ baseline ~0.06 m).
- 손목 카메라는 USD에서 읽은 RealSense **D405** 위치(`arm_*_link7/visuals/d405`): 로컬
  `pos (0.10683, 0, -0.07713)`, Y축 180°.
- 카메라는 로봇 링크의 자식이라 **렌더/녹화 피드가 헤드/팔을 따라 움직인다**(스테이지 아이콘만
  고정돼 보이는 건 시각적 특성).
- **외부 파라미터는 보정 전 placeholder** — 교차 도메인 학습 전 실물과 대조 필요.

### 9.2 베이스 속도 **[추가]**

mobile 태스크는 `obs/base_velocity = base_planar_velocity(env)` =
`[root_lin_vel_b.x, root_lin_vel_b.y, root_ang_vel_b.z]`(base 프레임)을 기록. 변환기가 이를
state와 action에 붙여 22차원으로, 실물과 일치시킨다.

---

## 10. 주행 가능 모바일 베이스 **[추가]**

stock `FFW_SG2.usd`는 정지 매니퓰레이션용이다: `FixedJoint`가 섀시를 월드에 용접, 휠 drive
조인트가 ±1080°에서 정지, 좌/우 휠 콜라이더 꺼짐, 중력 꺼짐. 베이스는 바퀴 물리가 아니라
키네마틱 root 텔레포트로 "이동"한다.

`FFW_SG2_MOBILE`은 stock USD를 참조하는 `~2 KB` 오버라이드 레이어에서 이 잠금들을 푼다(stock
자산은 절대 수정 안 함):

| stock 잠금 | 해결 |
|---|---|
| `FixedJoint`로 섀시 용접 | `fix_root_link=False` (자유 베이스) |
| 휠 drive 제한 ±1080° | 제거 (연속 회전) |
| 좌/우 휠 콜라이더 꺼짐 | 다시 켬 |
| 중력 꺼짐 | **바디별**: 베이스+휠 6개 ON(접지력), 팔/리프트/헤드/그리퍼 OFF(처짐 방지) |
| — | 자기충돌 **ON**(팔이 몸통 관통 방지); 휠 6개는 모든 몸체 링크에 필터링(휠은 바닥만 충돌) |
| — | reset 이벤트가 `reset_scene_to_default` 후 베이스를 정상 높이로 올림 |

녹화 중 작업자가 `/cmd_vel`로 베이스를 몰면 SDK가 스워브 휠 타겟으로 적용한다
(`_apply_swerve_cmd_vel(..., integrate_root=False)` → 물리 주행). 검증: `root_z ≈ 1.405`로 안정,
10초에 8.23 m 주행(지령 속도의 96%), 홀로노믹 게걸음/제자리회전 확인.

도구: `scripts/tools/build_ffw_sg2_mobile_usd.py`(USD 재생성),
`check_ffw_sg2_mobile.py`(6/6 회귀), `teleop_sg2_mobile.py`(키보드 주행).

---

## 11. 알려진 제약

- **Mimic / datagen은 아직 mobile 22차원 데이터용으로 미배선.** 녹화·LeRobot 내보내기는 22차원
  처리 가능하지만, 증강 파이프라인(IK 변환 → annotate → datagen)은 아직 19차원 경로 대상.
- **카메라 외부 파라미터는 placeholder** — 실물과 대조해 보정 필요.
- **프레임드랍**은 GPU 포화 시 카메라 4개로 조작감에 영향을 주지만 **녹화 데이터엔 영향 없음**
  (각 프레임은 해당 시뮬 시점의 올바른 이미지; 시뮬이 실시간보다 느리게 돌 뿐).
- **action의 베이스 속도는 측정 twist**를 명령 대용으로 사용(스워브가 `/cmd_vel`을 잘 추종).

---

## 부록 A — 추가 / 수정 파일

모든 경로는 `overlays/`(버전관리 오버레이) 아래이며, `setup.sh` / `sync_overlay.sh`가 라이브
체크아웃에 적용한다.

### 추가

| 파일 | 용도 |
|---|---|
| `cyclo_lab/…/assets/robots/FFW_SG2_MOBILE.py` | 주행 가능 베이스 아티큘레이션 config |
| `cyclo_lab/…/data/robots/FFW/FFW_SG2_MOBILE.usd` | stock USD 위 ~2 KB 오버라이드 레이어 |
| `cyclo_lab/…/controllers/swerve.py` (+ `__init__.py`) | 3모듈 홀로노믹 스워브 IK 컨트롤러 |
| `cyclo_lab/scripts/tools/build_ffw_sg2_mobile_usd.py` | `FFW_SG2_MOBILE.usd` 재생성 |
| `cyclo_lab/scripts/tools/check_ffw_sg2_mobile.py` | 주행 / 홀로노믹 회귀 검증 (6개) |
| `cyclo_lab/scripts/tools/teleop_sg2_mobile.py` | 키보드 베이스 주행 (개발용) |
| `DEPLOY_PLAN_B.md` | 모바일 녹화 배포 & 테스트 절차 |

### 수정

| 파일 | 변경 내용 |
|---|---|
| `cyclo_lab/…/pick_place_l_table/joint_pos_env_cfg.py` | 카메라 4개; `FFWSG2PickPlaceLTableMobileEnvCfg`; 베이스 속도 관측; reset 이벤트 |
| `cyclo_lab/…/pick_place_l_table/pick_place_env_cfg.py` | 카메라 씬 슬롯 + 관측항; teleop 플래그 |
| `cyclo_lab/…/pick_place_l_table/__init__.py` | mobile 태스크 등록 |
| `cyclo_lab/…/pick_place_l_table/mdp/observations.py` | `base_planar_velocity` 관측 |
| `cyclo_lab/…/pick_place_l_table/mdp/ffw_sg2_l_table_events.py` | `reset_mobile_base_standing` 이벤트 |
| `cyclo_lab/scripts/…/data_converter/isaaclab2lerobot.py` | 카메라 자동 감지(rgb, CHW) + 베이스 속도 → 22차원 |
| `cyclo_lab/scripts/…/dds_sdk/ffw_sg2_sdk.py` | `/cmd_vel` 구독, 물리 스워브 베이스 주행 |
| `cyclo_lab/sg2_ltable_dashboard.py` | 세션 기반 데이터셋 네이밍; 태스크 목록에 mobile 추가 |
| `robotis_applications/robotis_vuer/robotis_vuer/vr_publisher_sg2.py` | Y 버튼 모드 토글; 두 모드 A/B 베이스 회전; `/cmd_vel` → RELIABLE QoS |

---

*베이스 레포: [`EKAIWORKER`](https://github.com/Disniekie01/EKAIWORKER) · Fork:
[`AI_HUN`](https://github.com/hun7407-lgtm/AI_HUN). 업스트림: ROBOTIS `cyclo_lab`, `ai_worker`,
`robotis_applications` (`setup.sh`에 핀).*
