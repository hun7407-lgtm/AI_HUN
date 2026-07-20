# 데이터 수집 파이프라인 — FFW-SG2 VR 원격조작

**작성자:** Hun Kim (hun7728@hanyang.ac.kr) · **수정일:** 2026-07-20

NVIDIA Isaac Sim에서 Meta Quest VR 원격조작으로 **양팔 매니퓰레이션 + 모바일 베이스**
시연을 수집하고, 실제 `ffw_sg2_rev1` 로봇의 데이터 스키마와 일치하는 **LeRobot** 포맷으로
내보내는 전 과정 가이드.

이 문서는 실제 데이터 수집이 진행되는 순서대로 정렬되어 있다:
**셋업 → 환경 → 실물 정합 → 원격조작 → 녹화 → 변환**. 업스트림 ROBOTIS 스택 위에
추가한 작업은 **[추가]** 또는 **[수정]** 으로 표시했고, 건드린 파일은 전부
[부록 A](#부록-a--추가--수정-파일)에 정리했다.

---

## 1. 개요

| | |
|---|---|
| **목표** | 실제 로봇과 동일한 포맷으로 시연을 수집 → sim/real 데이터를 함께 모아 모방학습에 사용 |
| **로봇** | ROBOTIS FFW-SG2 — 양팔 7-DoF + 평행 그리퍼, 헤드(2-DoF), 수직 리프트, 3모듈 스워브 베이스 |
| **태스크** | L-테이블 pick & place (앞 테이블의 박스를 집어 옮겨 왼쪽 테이블에 놓기) |
| **엔진** | Isaac Sim **5.1.0** + Isaac Lab **2.3.0** |
| **입력** | Meta Quest 3 착용 작업자 (WebXR / Vuer) — 팔 원격조작 **+ 베이스 주행** |
| **출력** | LeRobot 데이터셋 — RGB 카메라 4개 + 22차원 state/action (관절 19 + 베이스 속도) |

### 데이터 흐름

```
[작업자 + Meta Quest 3]
      │  손/컨트롤러 포즈 + 조이스틱 (WebXR)
      ▼
[robotis-applications] Vuer VR 퍼블리셔 ──/cmd_vel + 팔 포즈 (ROS 2 / DDS, domain 30)──┐
      │                                                                                │
      ▼                                                                                ▼
[ai_worker] vr_controller (팔 IK)                                       [cyclo_lab] FFWSG2Sdk
      │  관절 명령 (DDS)                                                     스워브 베이스 주행
      ▼                                                                                │
[cyclo_lab] Isaac Sim + record_demos.py ◄────────────────────────────────────────────┘
      │  관측 (관절 + 베이스 속도 + 카메라 4개)
      ▼
   raw .hdf5  ──►  IK 변환 ──►  annotate ──►  (Mimic datagen)  ──►  joint 변환  ──►  LeRobot
```

---

## 2. 시스템 셋업

### 2.1 사전 준비 (`setup.sh`가 안 해주는 것)

| 요구사항 | 확인 |
|---|---|
| Linux 호스트 (Ubuntu / Pop!_OS) + X11 | — |
| NVIDIA RTX GPU + 드라이버 (검증: RTX PRO 5000, 24 GB) | `nvidia-smi` |
| Docker Engine + NVIDIA Container Toolkit | `docker run --gpus all …` |
| NGC 로그인 (`nvcr.io/nvidia/isaac-sim:5.1.0` pull) | `docker login nvcr.io` |
| Meta Quest 3 (+ USB 테더링용 최초 1회 ADB/udev) | `adb_vr_connect/README.md` |

### 2.2 클론 & 빌드

```bash
git clone https://github.com/hun7407-lgtm/AI_HUN.git AIWORKER
cd AIWORKER
./setup.sh ~/AIWORKER        # 업스트림 3개를 고정 커밋으로 클론 + 오버레이 적용
```

`setup.sh`는 `~/AIWORKER/{cyclo_lab, ai_worker, robotis_applications}`를 만든다. 이 레포는
**오버레이(overlay)** — 변경된 파일만 `overlays/`에 버전관리하고, 업스트림 원본과 녹화
데이터셋은 제외한다.

### 2.3 컨테이너 3개 시작

```bash
cd ~/AIWORKER/cyclo_lab/docker            && ./container.sh start   # Isaac Sim, 녹화, 대시보드 :8765
cd ~/AIWORKER/robotis_applications/docker && ./container.sh start   # Vuer VR 퍼블리셔 :8012
cd ~/AIWORKER/ai_worker/docker            && ./container.sh start   # 팔 IK 컨트롤러
```

셋 다 `ROS_DOMAIN_ID=30` (DDS / `rmw_fastrtps_cpp`)을 공유한다.

---

## 3. 시뮬레이션 환경

L-테이블 태스크에 두 가지 변형이 있다. **stock 태스크는 그대로 두고**, 베이스 속도 수집용
mobile 태스크를 **[추가]** 했다.

| 태스크 ID | 베이스 | 베이스 이동 | state/action | 용도 |
|---|---|---|---|---|
| `Cyclo-Real-Pick-Place-LTable-FFW-SG2-v0` | 고정(용접) | scripted L-motion (키네마틱 텔레포트) | 19차원 | stock (팔만) |
| `Cyclo-Real-Pick-Place-LTable-Mobile-FFW-SG2-v0` **[추가]** | 자유(주행) | `/cmd_vel`로 작업자가 주행 (물리 스워브) | 22차원 | 모바일 베이스 수집 |

---

## 4. 실물 정합: 카메라 **[수정]**

실물 `ffw_sg2_rev1`은 RGB 스트림 **4개**를 기록하는데, stock sim은 헤드 카메라 1개만
렌더했다. 녹화가 실물 관측 스키마와 맞도록 카메라 3개를 추가했다.

| LeRobot 키 | Sim 카메라 | 마운트 | 해상도 |
|---|---|---|---|
| `observation.images.rgb.cam_left_head` | `cam_head` (ZED 좌안) | `head_link2/zed` | 376 × 672 |
| `observation.images.rgb.cam_right_head` | `cam_right_head` | `head_link2/zed` (대칭) | 376 × 672 |
| `observation.images.rgb.cam_left_wrist` | `cam_left_wrist` | `arm_l_link7` (D405 위치) | 424 × 240 |
| `observation.images.rgb.cam_right_wrist` | `cam_right_wrist` | `arm_r_link7` (D405 위치) | 424 × 240 |

핵심:
- **헤드 카메라**는 헤드 ZED의 좌/우 눈이며, `zed` prim 중심 기준 Y축 대칭(±0.03 m ⇒
  baseline ~0.06 m)으로 배치.
- **손목 카메라**는 USD에서 직접 읽은 RealSense **D405** 위치(`arm_*_link7/visuals/d405`):
  로컬 `pos (0.10683, 0, -0.07713)`, Y축 180°.
- 카메라는 로봇 링크의 자식이라 렌더 시 **헤드/팔을 따라 움직인다** (녹화되는 피드는 링크를
  추종. 스테이지의 카메라 아이콘만 고정돼 보이는데 이는 Isaac Sim의 시각적 특성).
- **카메라 외부 파라미터는 보정 전 placeholder** — 교차 도메인 학습 전에 실물과 대조 필요.

> 수정: `pick_place_l_table/joint_pos_env_cfg.py` (카메라 정의),
> `pick_place_env_cfg.py` (씬 슬롯 + 관측항), `mdp/observations.py`.

---

## 5. 주행 가능 모바일 베이스 **[추가]**

stock `FFW_SG2.usd`는 정지 매니퓰레이션용이다: `FixedJoint`가 섀시를 월드에 용접, 휠 drive
조인트가 ±1080°로 제한, 좌/우 휠 콜라이더 꺼짐, 중력 꺼짐. stock 태스크의 베이스 이동은
바퀴 물리가 아니라 **키네마틱 root 텔레포트**다.

`FFW_SG2_MOBILE`은 이 잠금들을 풀어 베이스가 물리로 주행하게 한다:

| stock 잠금 | 주행 변형에서의 해결 |
|---|---|
| `FixedJoint`로 섀시를 월드에 용접 | `fix_root_link=False` (자유 베이스) |
| 휠 drive 제한 ±1080° (~1.63 m) | 제한 제거 (연속 회전) |
| 좌/우 휠 콜라이더 꺼짐 | 다시 켬 (바퀴 3개 모두 지면 접촉) |
| 중력 꺼짐 | **바디별**: 베이스+휠 6개는 ON(접지력), 팔/리프트/헤드/그리퍼는 OFF(처짐 방지) |
| — | 자기충돌 **ON** (팔이 몸통 관통 방지), 휠 6개는 모든 몸체 링크에 대해 필터링(휠은 바닥만 충돌) |
| — | reset 이벤트가 `reset_scene_to_default` 후 베이스를 정상 높이로 올림 |

주행 로봇은 stock 41 MB USD를 **참조**하는 `~2 KB` 오버라이드 레이어다 (stock 자산은 절대
수정 안 함). 검증: 바퀴 위에 `root_z ≈ 1.405`로 안정 착지, 10초에 8.23 m 주행(지령 속도의
96%), 홀로노믹 게걸음/제자리회전 확인.

> 추가: `assets/robots/FFW_SG2_MOBILE.py`, `data/robots/FFW/FFW_SG2_MOBILE.usd`,
> `controllers/swerve.py` (3모듈 스워브 IK), `scripts/tools/build_ffw_sg2_mobile_usd.py`
> (USD 재생성), `scripts/tools/check_ffw_sg2_mobile.py` (6개 회귀 검증),
> `scripts/tools/teleop_sg2_mobile.py` (키보드 주행).

**주행 USD 재생성 / 검증:**

```bash
# cyclo_lab 컨테이너 안, /workspace/cyclo_lab 에서
./third_party/IsaacLab/isaaclab.sh -p scripts/tools/build_ffw_sg2_mobile_usd.py --force
./third_party/IsaacLab/isaaclab.sh -p scripts/tools/check_ffw_sg2_mobile.py --headless   # 6/6 기대
```

---

## 6. 베이스 주행 VR 원격조작 **[수정]**

녹화 중 작업자는 팔을 원격조작하면서 **동시에** Quest 조이스틱으로 베이스를 몬다. 베이스는
`/cmd_vel`(`Twist`) 발행으로 구동되며, sim이 이를 소비해 스워브 바퀴를 물리로 구동한다
(`integrate_root=False`).

### 6.1 조작

| 입력 | 동작 |
|---|---|
| 양쪽 그립 ~3초 홀드 | 팔 teleop 활성 (기존과 동일) |
| **왼쪽 Y 버튼** | 베이스 모드 토글: `LIFT+HEAD` ↔ `LIFT+CMD_VEL` **[추가]** |
| 왼쪽 썸스틱 (`CMD_VEL` 모드) | 베이스 이동 (전후, 게걸음) |
| **A 버튼 (홀드)** | 베이스 **우회전** (`angular.z < 0`) — 두 모드 다 작동 **[추가]** |
| **B 버튼 (홀드)** | 베이스 **좌회전** (`angular.z > 0`) — 두 모드 다 작동 **[추가]** |
| 오른쪽 썸스틱 X | 리프트 상승/하강 |

참고:
- A + B 동시 입력은 상쇄됨 (우 − 좌 = 0); 하나씩 누를 것.
- Vuer 매핑상 각 컨트롤러는 `aButton`/`bButton`을 노출한다. 물리 **Y**는 **왼쪽**
  컨트롤러의 `bButton`, 회전 A/B는 **오른쪽** 컨트롤러 버튼이다.
- 시작/방향 전환 시 짧은 반응 지연은 정상적인 스워브 동작이다 (바퀴가 재조향 후 베이스가
  움직임). 시뮬이 실시간보다 느리면 이 지연이 더 커진다.

### 6.2 `/cmd_vel` 전송 수정 **[수정 — 핵심]**

베이스 명령이 **DDS QoS 불일치**로 조용히 버려지고 있었다: VR 퍼블리셔가 `/cmd_vel`을
`BEST_EFFORT`로 보냈는데 sim SDK는 `RELIABLE`로 구독했고, DDS에서 `RELIABLE` 구독자는
`BEST_EFFORT` 퍼블리셔와 매칭되지 않는다. `/cmd_vel`을 (작동하는 팔/리프트/헤드 명령
토픽이 쓰는) `RELIABLE` `vr_command_qos`로 바꿨다.

> 수정: `robotis_applications/robotis_vuer/vr_publisher_sg2.py` (Y 토글, 두 모드 A/B 회전,
> `/cmd_vel` → RELIABLE), `dds_sdk/ffw_sg2_sdk.py` (`/cmd_vel` 구독, 매 스텝 스워브 주행
> 적용), `pick_place_l_table/__init__.py` (mobile 태스크 등록),
> `mdp/ffw_sg2_l_table_events.py` (`reset_mobile_base_standing`).

배포 주의: VR 퍼블리셔는 symlink-install이라, 수정 후 VR 노드를 재시작해야(대시보드
**Launch VR + Controller**) 반영된다. `DEPLOY_PLAN_B.md` 참고.

---

## 7. 시연 녹화

### 7.1 실행

```bash
cd ~/AIWORKER/cyclo_lab
python3 sg2_ltable_dashboard.py          # http://localhost:8765
```

1. 태스크 **"L-Table Pick & Place (mobile base)"** 선택 **[대시보드에 추가됨]**.
2. **Launch Record** → Isaac Sim 실행; 로봇이 바퀴 위에 섬.
3. Quest 연결 (USB/ADB 권장) 후 Enter VR.
4. **Y** 버튼으로 `LIFT+CMD_VEL` 모드 진입 → 베이스 주행.

### 7.2 녹화 조작 (Isaac 창 포커스)

| 키 | 동작 |
|---|---|
| **B** | 이 take 녹화 시작 + 팔 teleop |
| **N** | 에피소드 저장 |
| **R** | 폐기 후 재시작 |

### 7.3 세션 기반 데이터셋 네이밍 **[수정]**

대시보드는 동시 작업자 관리가 쉽도록 세션별 레이아웃으로 데이터셋을 저장한다:

```
datasets/{base}/{YYYYMMDD}/{HHMM}_{ip}_{src}_{base}_{stage}.hdf5
#   ip = 호스트 IPv4 마지막 옥텟   src = 'vr' | 'leader'   stage = raw|ik|annotate|gen|joint
```

대시보드에서 `USE_SESSION_NAMING = False`로 하면 기존 flat 네이밍
(`datasets/<base>_<stage>.hdf5`)으로 되돌아간다.

> 수정: `sg2_ltable_dashboard.py`.

---

## 8. LeRobot 변환 **[수정]**

변환기는 녹화 내용을 자동 감지해서 실물 스키마로 내보낸다 — 별도 플래그 불필요:

```bash
# cyclo_lab 안, /workspace/cyclo_lab 에서
lerobot-python scripts/sim2real/imitation_learning/data_converter/isaaclab2lerobot.py \
  --task Cyclo-Real-Pick-Place-LTable-Mobile-FFW-SG2-v0 \
  --robot_type FFW_SG2 --fps 15 \
  --dataset_file ./datasets/<...>_joint.hdf5
```

동작:
- **카메라** — 녹화된 카메라 스트림을 감지해 각각 `observation.images.rgb.<name>`,
  **채널 우선 `[3, H, W]`** 로 내보냄 (실물 데이터셋과 일치). `cam_head` → `cam_left_head`;
  해상도는 녹화 배열에서 읽음.
- **베이스 속도** — `obs/base_velocity`가 있으면(mobile 태스크) `observation.state`와
  `action`에 `[linear_x, linear_y, angular_z]`를 붙여 **22차원**으로; 없으면 19차원 유지
  (stock 태스크 무손상).
- 로그에 `Base velocity: present -> 22-dim state/action`과 감지된 카메라가 찍힘.

> 수정: `data_converter/isaaclab2lerobot.py`. 베이스 속도 관측은
> `mdp/observations.py`(`base_planar_velocity`)에 추가, mobile 태스크의 policy 그룹에
> `joint_pos_env_cfg.py`에서 배선.

---

## 9. 출력 데이터 스키마 (실물 `ffw_sg2_rev1`과 일치)

| Feature | Shape | 비고 |
|---|---|---|
| `observation.state` | (22,) | 관절 19 + `linear_x, linear_y, angular_z` |
| `action` | (22,) | 동일 레이아웃 (베이스 부분 = 측정 twist) |
| `observation.images.rgb.cam_left_head` | [3, 376, 672] | video, CHW |
| `observation.images.rgb.cam_right_head` | [3, 376, 672] | video, CHW |
| `observation.images.rgb.cam_left_wrist` | [3, 424, 240] | video, CHW |
| `observation.images.rgb.cam_right_wrist` | [3, 424, 240] | video, CHW |

관절 19개: `arm_l_joint1..7, gripper_l_joint1, arm_r_joint1..7, gripper_r_joint1,
head_joint1, head_joint2, lift_joint`.

---

## 10. 알려진 제약

- **Mimic / datagen은 아직 mobile 22차원 데이터용으로 미배선** — 녹화와 LeRobot 변환은
  지원. 합성 증강 파이프라인(IK 변환 → annotate → datagen)은 아직 19차원 경로 대상.
- **카메라 외부 파라미터는 placeholder** — 실물과 대조해 보정 필요.
- **프레임드랍**은 GPU 포화 시 카메라 4개로 조작감에 영향을 주지만 **녹화 데이터엔 영향
  없음** (각 프레임은 해당 시뮬 시점의 올바른 이미지).
- **action의 베이스 속도는 측정 twist**를 명령 대용으로 사용 (스워브가 `/cmd_vel`을 잘
  추종).

---

## 부록 A — 추가 / 수정 파일

모든 경로는 `overlays/`(버전관리 오버레이) 아래이며, `setup.sh` / `sync_overlay.sh`가 라이브
체크아웃에 rsync한다.

### 추가

| 파일 | 용도 |
|---|---|
| `cyclo_lab/source/cyclo_lab/cyclo_lab/assets/robots/FFW_SG2_MOBILE.py` | 주행 가능 베이스 아티큘레이션 config |
| `cyclo_lab/source/cyclo_lab/data/robots/FFW/FFW_SG2_MOBILE.usd` | stock USD 위 ~2 KB 오버라이드 레이어 |
| `cyclo_lab/source/cyclo_lab/cyclo_lab/controllers/swerve.py` (+ `__init__.py`) | 3모듈 홀로노믹 스워브 IK 컨트롤러 |
| `cyclo_lab/scripts/tools/build_ffw_sg2_mobile_usd.py` | `FFW_SG2_MOBILE.usd` 재생성 |
| `cyclo_lab/scripts/tools/check_ffw_sg2_mobile.py` | 주행/홀로노믹 회귀 검증 (6개) |
| `cyclo_lab/scripts/tools/teleop_sg2_mobile.py` | 키보드 베이스 주행 (개발용) |
| `DEPLOY_PLAN_B.md` | 모바일 녹화 배포 & 테스트 절차 |

### 수정

| 파일 | 변경 내용 |
|---|---|
| `cyclo_lab/.../pick_place_l_table/joint_pos_env_cfg.py` | 카메라 4개; `FFWSG2PickPlaceLTableMobileEnvCfg`; 베이스 속도 관측; reset 이벤트 |
| `cyclo_lab/.../pick_place_l_table/pick_place_env_cfg.py` | 카메라 씬 슬롯 + 관측항; teleop 플래그 |
| `cyclo_lab/.../pick_place_l_table/__init__.py` | mobile 태스크 등록 |
| `cyclo_lab/.../pick_place_l_table/mdp/observations.py` | `base_planar_velocity` 관측 |
| `cyclo_lab/.../pick_place_l_table/mdp/ffw_sg2_l_table_events.py` | `reset_mobile_base_standing` 이벤트 |
| `cyclo_lab/scripts/sim2real/imitation_learning/data_converter/isaaclab2lerobot.py` | 카메라 자동 감지(rgb, CHW) + 베이스 속도 → 22차원 |
| `cyclo_lab/scripts/sim2real/imitation_learning/dds_sdk/ffw_sg2_sdk.py` | `/cmd_vel` 구독, 물리 스워브 베이스 주행 |
| `cyclo_lab/sg2_ltable_dashboard.py` | 세션 기반 데이터셋 네이밍; 태스크 목록에 mobile 추가 |
| `robotis_applications/robotis_vuer/robotis_vuer/vr_publisher_sg2.py` | Y 버튼 모드 토글; 두 모드 A/B 베이스 회전; `/cmd_vel` → RELIABLE QoS |
