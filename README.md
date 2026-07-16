# AI_HUN — VR Teleoperation 데이터 수집 오버레이

> **AI Worker (ROBOTIS FFW) · Isaac Sim VR 원격조작(teleoperation) 데이터 수집·증강 파이프라인**
> 사람이 Meta Quest 3로 로봇을 조종 → Isaac Sim에서 시연 녹화 → IsaacLab **Mimic**으로 대량 증강 → 학습용 **HDF5** 산출.

이 레포는 **"오버레이(overlay)" 레포**입니다. ROBOTIS 원본 저장소 전체를 담지 않고, 녹화·증강용 로컬 수정본만 `overlays/`에 담습니다. `setup.sh`가 원본 3개를 **고정 커밋(pinned commit)** 으로 클론한 뒤, 그 위에 이 레포의 오버레이를 덮어씁니다.

- **레포 크기가 작은 이유**: 업스트림 원본과 녹화 데이터셋(수백 GB)은 **커밋에서 제외**됩니다. 클론한 머신에서 `setup.sh`로 재구성합니다.
- **English TL;DR**: This is a thin *overlay* repo. `setup.sh` clones 3 upstream ROBOTIS repos at pinned commits, then rsyncs `overlays/` on top. Upstream sources and recorded HDF5 datasets are intentionally **not committed**.

---

## 1. 전체 흐름 (End-to-End Flow)

```
[사람 + Meta Quest 3]
      │  손/컨트롤러 포즈 (WebXR)
      │  · WiFi:  vuer.ai  ↔  wss://<PC-IP>:8012
      │  · USB :  adb reverse  ↔  wss://localhost:8012   (지터 낮음, 녹화 권장)
      ▼
[robotis-applications 컨테이너]  Vuer VR Publisher  (:8012)
      │  ROS 2 포즈 토픽 (DDS / FastRTPS, ROS_DOMAIN_ID=30)
      ▼
[ai_worker 컨테이너]  vr_controller (IK 모션 컨트롤러)
      │  관절 명령 (DDS)
      ▼
[cyclo_lab 컨테이너]  Isaac Sim 5.1.0 + IsaacLab 2.3.0 + record_demos.py
      │  시연 녹화
      ▼
   *_raw.hdf5  ──► IK 변환 ──► Annotate ──► Mimic Datagen(500개 증강) ──► Joint 변환 ──► (LeRobot)
                                                                              │
                                                                              ▼
                                                                     학습용 HDF5 데이터셋
```

전 과정은 **`cyclo_lab` 대시보드(`http://localhost:8765`)** 에서 버튼으로 오케스트레이션됩니다.

---

## 2. 버전 스펙 (Version Spec) ⭐

세팅 시 **이 버전 조합**을 맞춰야 태스크 등록/IsaacLab API가 호환됩니다.

| 구성요소 | 버전 | 비고 |
|---|---|---|
| **NVIDIA Isaac Sim** | **5.1.0** | 물리·렌더 엔진. `docker/.env`의 `ISAACSIM_VERSION=5.1.0`으로 핀. 베이스 이미지 `nvcr.io/nvidia/isaac-sim` |
| **NVIDIA Isaac Lab** | **2.3.0** | 태스크 정의·Recorder Manager·HDF5 데이터셋 프레임워크. `cyclo_lab/third_party/IsaacLab` 서브모듈 |
| ├ `isaaclab` | 0.47.2 | 코어 |
| ├ `isaaclab_mimic` | 1.0.15 | **데이터 증강(datagen)** |
| ├ `isaaclab_tasks` | 0.11.6 | 태스크 |
| ├ `isaaclab_rl` | 0.4.4 | RL |
| └ `isaaclab_assets` | 0.2.3 | 에셋 |
| **ROS 2** | **Jazzy Jalisco** | 컨테이너 간 통신. RMW = `rmw_fastrtps_cpp` (Fast DDS), `ROS_DOMAIN_ID=30` |
| **Vuer (WebXR)** | 컨테이너 내장 | VR 브릿지, 포트 **8012** |
| **Meta Quest 3** | — | WebXR 브라우저로 접속 (앱 설치 불필요) |

### 컨테이너 이미지 (Docker)

| 컨테이너 | 이미지 | 역할 | 포트 |
|---|---|---|---|
| `cyclo_lab` | `cyclolab/cyclo-lab:latest` (Isaac Sim 5.1.0 base) | Isaac Sim·녹화·Mimic 파이프라인·대시보드 | **8765** |
| `robotis-applications` | `robotis/robotis-applications:1.0.0` | Vuer VR Publisher | **8012** |
| `ai_worker` | `robotis/ai-worker:2.0.0` | vr_controller (IK 모션) | — |

### 검증된 하드웨어 (Reference HW)

| 항목 | 사양 |
|---|---|
| CPU | Intel Core Ultra 9 285HX |
| GPU | **NVIDIA RTX PRO 5000 Blackwell Laptop (24GB)** |
| OS | Ubuntu / Pop!_OS (Linux) + NVIDIA Container Toolkit |

### 업스트림 고정 커밋 (Upstream Pins — `setup.sh` 기준)

| 원본 | 커밋 |
|---|---|
| `cyclo_lab` | `a5ea01967b145f839ca1ac8f51b42abf9ef87036` |
| `ai_worker` | `e8c2eacb612e47473cdf03e44bee6d527c00b4f9` |
| `robotis_applications` | `7ef0aabc748174cb91013866b2e4142122ef475c` |

---

## 3. 클론 & 설치 (Clone & Setup)

### 3-0. 사전 준비 (Prerequisites) — 신규 머신 필수 ⭐

> `setup.sh`는 **코드만** 재현합니다. 아래 하드웨어·환경은 **미리** 갖춰져 있어야 컨테이너가 뜹니다. `git clone`만으로는 안 됩니다.

| 전제조건 | 설명 | 확인/설치 |
|---|---|---|
| **Linux 호스트** | Ubuntu / Pop!_OS 등 (검증: Linux + X11) | — |
| **NVIDIA GPU + 드라이버** | RTX급 필수 (Isaac Sim이 무거움). 검증HW = RTX PRO 5000 24GB | `nvidia-smi` |
| **Docker Engine** | 컨테이너 런타임 | `docker --version` |
| **NVIDIA Container Toolkit** | 컨테이너가 GPU를 쓰게 함(nvidia-docker). **없으면 Isaac Sim 안 뜸** | `docker run --rm --gpus all nvidia/cuda:12.4.0-base-ubuntu22.04 nvidia-smi` |
| **NGC 계정 + 로그인** | `container.sh start`가 `nvcr.io/nvidia/isaac-sim:5.1.0` 베이스를 pull해서 `Dockerfile.base`로 빌드함 → NGC 인증 필요 + EULA 동의 | `docker login nvcr.io` (Username: `$oauthtoken`, Password: NGC API 키) |
| **디스크 용량** | 이미지 빌드 수십 GB + 녹화 데이터셋 수백 GB | `df -h` |
| **Meta Quest 3 + ADB/udev** | USB 테더링 최초 1회 세팅 | [`adb_vr_connect/README.md`](adb_vr_connect/README.md) |

> **참고**: 이 레포엔 녹화 데이터셋·학습 체크포인트가 없습니다(의도적 제외). 신규 머신에서는 **직접 녹화**해서 데이터를 새로 수집합니다.

설치 요약 순서:

```text
[0] 사전 준비:  NVIDIA 드라이버 + Docker + NVIDIA Container Toolkit 설치, docker login nvcr.io
[1] 코드:       git clone AI_HUN → ./setup.sh ~/AIWORKER
[2] 컨테이너:    각 docker/ 에서 ./container.sh start   (첫 실행은 이미지 빌드로 오래 걸림)
[3] 실행:       xhost +local:docker → python3 sg2_ltable_dashboard.py → http://localhost:8765
```

### 3-1. 이 레포 클론 + 원본 자동 구성

```bash
# 1) 이 레포(오버레이) 클론
git clone https://github.com/hun7407-lgtm/AI_HUN.git AIWORKER
cd AIWORKER

# 2) setup.sh 실행: 원본 3개를 고정 커밋으로 클론 + 서브모듈 초기화 + 오버레이 적용
./setup.sh ~/AIWORKER
```

`setup.sh ~/AIWORKER` 실행 후 생성되는 구조:

```text
~/AIWORKER/
  cyclo_lab/              ← 원본 클론 + 오버레이 적용 (Isaac Sim/녹화/Mimic/대시보드)
  ai_worker/              ← 원본 클론 (vr_controller)
  robotis_applications/   ← 원본 클론 + 오버레이 적용 (Vuer)
```

> 설치 경로는 자유롭게 지정 가능 (`./setup.sh /원하는/경로`).

### 3-2. 컨테이너 시작

```bash
cd ~/AIWORKER/cyclo_lab/docker            && ./container.sh start
cd ~/AIWORKER/robotis_applications/docker && ./container.sh start
cd ~/AIWORKER/ai_worker/docker            && ./container.sh start
```

### 3-3. GUI 허용 (호스트 터미널)

```bash
xhost +local:docker
xhost +SI:localuser:root
# Isaac 창이 안 뜨면: xhost +
```

### 3-4. 대시보드 시작

```bash
cd ~/AIWORKER/cyclo_lab
python3 sg2_ltable_dashboard.py
```

브라우저에서 **`http://localhost:8765`** 접속 → Robot·Task 선택 → **Launch VR + Controller** (또는 **Launch Record**).

---

## 4. VR 연결 (Meta Quest 3)

대시보드가 스택을 실행하면, Quest 브라우저에서 Vuer(WebXR)로 접속합니다.

| 서비스 | 주소 | 용도 |
|---|---|---|
| 대시보드 | `http://localhost:8765` | 태스크 선택·스택 실행·Mimic 파이프라인 |
| Vuer (HTTPS) | `https://<PC-IP>:8012` | WebXR 페이지 |
| Vuer (WebSocket) | `wss://<PC-IP>:8012` | 포즈/버튼 스트림 (반드시 `wss://`) |

**WiFi — 2단계 (Quest 브라우저):**
1. 인증서 수락: `https://<PC-IP>:8012` → **Advanced → Proceed**
2. 새 탭: `https://vuer.ai/?ws=wss://<PC-IP>:8012` → **Enter VR** → 핸드트래킹 허용

**USB(ADB reverse) — 오프라인, 지터 낮음 (녹화 권장):**
```bash
cd ~/AIWORKER/adb_vr_connect && ./connect.sh   # 케이블 뽑을 때마다 재실행
```
Quest 브라우저: `https://localhost:8012?ws=wss://localhost:8012`
→ 최초 1회 ADB/udev 세팅은 [`adb_vr_connect/README.md`](adb_vr_connect/README.md) 참고.

**로봇 제어 활성화:**
- **SG2 (그리퍼)**: 양쪽 컨트롤러 그립을 **3초간 홀드** → teleop 활성화. 리프트는 오른쪽 썸스틱.
- **SH5 (핸드)**: VR 퍼블리싱이 **비활성**으로 시작 → 핸드 제스처 또는 `/vr/reactivate` 토픽. 리프트는 Isaac 창에서 **I**/**O**.

### 4-1. VR ↔ Isaac Sim 연결 안정화 루틴 (반복 체크리스트)

녹화 세션마다 아래 순서를 그대로 반복하면 안정적으로 연결됩니다 (USB/ADB 기준).

1. **USB로 PC ↔ VR 헤드셋 연결**
2. **연결 스크립트 실행** — `~/AIWORKER/adb_vr_connect/connect.sh`
3. 대시보드에서 **[Launch VR + Controller]** 버튼 입력
4. PC에서 **localhost(Vuer :8012) 실행 여부 확인**
5. Meta Quest 브라우저에서 **localhost 접속** — `https://localhost:8012?ws=wss://localhost:8012`
6. 대시보드 **[Launch Record]** 버튼 실행
7. **Isaac Sim 실행 후** VR 브라우저의 localhost **연결 상태 재확인**
8. Isaac Sim 창에서 **`B` 입력 → 컨트롤러 그립 약 3초 홀드**

> **연결 실패 시**: `R` 입력 → 다시 `B` 입력 → 그립 3초 홀드 절차를 반복.

**Task 수행 후 결과 확인:**
- 만족스러운 결과 → **`N`** 입력으로 데이터 저장
- 실패/불만족 → **`R`** 입력 후 재수행

---

## 5. 시연 녹화 (Recording) — Isaac 창 포커스

**매 take마다 반복:**

| 키 | 동작 |
|---|---|
| **B** | 녹화 시작 + arm teleop 활성 (**매 take 전에 필수**) |
| (VR) | SG2: 양쪽 그립 3초 홀드 → teleop on |
| **L** | 수동 L-motion (회전 후 L-테이블로 주행). 잡은 뒤 자동 트리거 가능 |
| **N** | 에피소드 저장 |
| **R** | take 폐기 후 재시작 (저장 안 됨) |

원본 데이터셋: `~/AIWORKER/cyclo_lab/datasets/*_raw.hdf5`
- SG2 액션 = 19차원 / SH5 액션 = 57차원 (양팔 + 손가락 20관절×2 + 헤드 + 리프트)

---

## 6. Mimic 파이프라인 (데이터 증강)

녹화한 `*_raw.hdf5`를 대시보드 **Mimic pipeline** 섹션에서 **한 번에 하나씩 순서대로** 실행 (각 단계 완료 후 다음):

| 단계 | 입력 → 출력 | 설명 |
|---|---|---|
| 1. IK convert | `raw → ik` | 관절 녹화를 IK(엔드이펙터) 액션으로 변환 |
| 2. Annotate | `ik → annotate` | 서브태스크 구간(grasp/move/place) 라벨링 |
| 3. **Datagen** | `annotate → generate` | **소수 시연 → 수백 개 합성** (기본 500개, `cyclo_mimic_datagen.py`) |
| 4. Joint convert | `generate → joint` | robomimic 학습용 관절 액션 HDF5 |
| 5. LeRobot export | `joint → lerobot` | (선택) ACT/LeRobot용 |

**튜닝 (대시보드 시작 전):**
```bash
export GENERATION_NUM_TRIALS=500   # datagen 에피소드 수 (기본 500)
export PIPELINE_NUM_ENVS=10        # datagen 병렬 Isaac 환경 수 (기본 10)
```

**단계별 로그:**
```bash
docker exec cyclo_lab tail -f /tmp/sg2_ltable_pipe_generate.log
# ik | annotate | generate | joint | lerobot
```

수동 실행(컨테이너 내부) 예시는 이 레포의 [`OPERATIONS.md`](OPERATIONS.md) 또는 상세 커맨드를 참고 (SG2/SH5 각각).

---

## 7. ⚠️ 주의사항 (Cautions)

- **버전 조합을 바꾸지 말 것**: Isaac Sim 5.1.0 / IsaacLab 2.3.0 조합이 검증됨. 업스트림 커밋을 바꾸면 태스크 등록·IsaacLab API가 어긋날 수 있으니 오버레이를 **반드시 재테스트**.
- **원본·데이터셋은 커밋 금지**: `cyclo_lab/`, `ai_worker/`, `robotis_applications/`(루트), `datasets/`, `*.hdf5`는 `.gitignore`로 제외됨. `git add -A` 시에도 절대 스테이징되지 않게 관리. (수백 GB)
- **오버레이가 진실의 원천(source of truth)**: 로컬 수정은 `overlays/`에서 관리. 라이브 체크아웃에 직접 수정했다면 `./pull_overlay.sh`로 되가져오고, 배포 시 `./sync_overlay.sh`로 덮어씀.
- **컨테이너 3개 모두 실행 필수**: VR(8012)·컨트롤러·녹화가 각각 다른 컨테이너. `ROS_DOMAIN_ID=30`으로 연결되므로 값이 어긋나면 서로 못 봄.
- **`wss://` 일치**: 페이지가 `https://`면 WebSocket도 반드시 `wss://` (혼합 콘텐츠 오류 방지).
- **녹화 중 지터**: WiFi 지터가 팔 떨림으로 나타남 → **USB ADB 테더링** 권장. GPU/CPU 전력·발열 스로틀도 프레임드랍 원인이 될 수 있음.
- **NVIDIA Docker 전제**: NVIDIA Container Toolkit 및 Isaac Sim 컨테이너 요구사항이 이미 충족되어 있다고 가정.
- **`docker/.env` 포함**: 개발 시 사용한 Isaac Sim 5.1 설정이 핀되어 있음.

---

## 7-1. 주행 가능 변형 (FFW_SG2_MOBILE) 🚗

스톡 `FFW_SG2.usd`는 **정지 상태 매니퓰레이션용**입니다. 팔·그리퍼 물리는 진짜지만, 모바일 베이스는 아닙니다:

| 스톡 자산의 제약 | 실측 |
|---|---|
| `FixedJoint`가 섀시(`world` 링크)를 월드에 **용접** | 물리적으로 주행 불가 |
| 휠 drive 조인트에 **±1080° 스톱** | 3바퀴 = **1.63 m** 주행 후 정지 |
| **좌/우 휠 충돌체 꺼짐** | 뒷바퀴 하나만 지면과 접촉 |
| `disable_gravity=True` | 접지 개념 없음 |

그래서 스톡 태스크의 베이스 이동은 바퀴 물리가 아니라 **kinematic root teleport**(`write_root_pose_to_sim`)입니다 — L-motion 주석에도 명시되어 있습니다.

`FFW_SG2_MOBILE`은 이 네 가지를 푼 변형입니다. **원본 USD와 스톡 태스크는 수정하지 않습니다** (녹화/datagen 그대로 동작).

```python
from cyclo_lab.assets.robots.FFW_SG2_MOBILE import FFW_SG2_MOBILE_CFG
from cyclo_lab.controllers import SwerveController

robot = Articulation(FFW_SG2_MOBILE_CFG.replace(prim_path="/World/Robot"))
swerve = SwerveController(robot)
swerve.apply(vx=0.5, vy=0.0, omega=0.0)        # 로봇 기준 (m/s, rad/s)
swerve.apply_world(vx=0.5, vy=0.0, omega=0.0)  # 월드 기준 (yaw 자동 보정)
swerve.stop()
robot.write_data_to_sim()
```

3륜 홀로노믹 스워브라 **전진·게걸음·제자리회전·대각선**이 모두 됩니다 (`vx`, `vy`, `ω` 3개 값 → IK가 바퀴 6개 지령으로 변환).

### 도구

```bash
# 키보드 조종 (GUI) — 화살표 전후/게걸음, Z/X 회전, E/Q 속도, C 기준전환, R 리셋
./third_party/IsaacLab/isaaclab.sh -p scripts/tools/teleop_sg2_mobile.py

# 회귀 검증 (6/6 통과해야 정상)
./third_party/IsaacLab/isaaclab.sh -p scripts/tools/check_ffw_sg2_mobile.py --headless

# USD 재생성 (data/robots/FFW/FFW_SG2_MOBILE.usd)
./third_party/IsaacLab/isaaclab.sh -p scripts/tools/build_ffw_sg2_mobile_usd.py --force
```

### 실측 성능

| 항목 | 결과 |
|---|---|
| 착지 | `root_z=1.4054`, 잔류속도 0.040 m/s |
| 직진 (10초) | **8.23 m**, 휠 98.7 rad(5654°) — 스톡 한계 1.63 m 돌파 |
| 게걸음 (3초) | dy=**+1.406 m**, dx=+0.018, dyaw=+1.3° |
| 제자리회전 (3초) | dyaw=**+125°**, 이동 0.032 m |
| 속도 효율 | 이론 대비 **96%** (나머지는 휠 슬립) |

### ⚠️ 주의

- **자기충돌은 켜면 안 됩니다.** 이 변형이 되살린 휠 콜라이더가 첫 프레임에 자기 하우징과 겹쳐, PhysX가 그 침투를 **로봇을 z=700 m로 발사**해서 해소합니다. wheel↔하우징 / wheel↔섀시 pair 필터링은 **불충분함이 측정으로 확인**됨. 대가: 팔이 몸통을 통과할 수 있음.
- **USD는 참조 레이어**(~2 KB)입니다. 41 MB 원본을 복제하지 않고 `./FFW_SG2.usd`를 참조하므로 **같은 폴더에 있어야** 합니다.
- **`default_root_state`로 리셋하지 마세요.** 그 값은 `init_state.pos=(0,0,0.01)`이지만 루트 바디의 실제 높이는 **1.4396**입니다(USD 내부 오프셋 1.43 m). 그대로 써넣으면 로봇이 땅속에 파묻혀 바퀴가 헛돕니다. 착지 후의 실제 자세를 저장해 두고 복귀시키세요.

---

## 8. 오버레이 동기화 (Overlay Sync)

```bash
# 오버레이 → 라이브 체크아웃에 적용
./sync_overlay.sh ~/AIWORKER/cyclo_lab

# 라이브 체크아웃 수정본 → 오버레이로 되가져오기 (역방향)
./pull_overlay.sh ~/AIWORKER/cyclo_lab
```

`manifest.json`의 업스트림 커밋은 부트스트랩 핀입니다. 실제 teleop 수정본은 `overlays/`에 담깁니다.

---

## 9. 레포 구조 (Repo Layout)

```text
AI_HUN/
├── setup.sh              # 원본 3개 클론(핀) + 서브모듈 + 오버레이 적용
├── sync_overlay.sh       # 오버레이 → 라이브 적용
├── pull_overlay.sh       # 라이브 → 오버레이 회수
├── run_dashboard.sh      # 대시보드 실행 헬퍼
├── manifest.json         # 업스트림 URL·커밋 핀
├── overlays/
│   ├── cyclo_lab/          # 녹화·Mimic·대시보드·태스크/에셋 수정본
│   └── robotis_applications/  # Vuer 퍼블리셔 수정본
└── adb_vr_connect/        # Meta Quest 3 USB(ADB) 테더링 (connect.sh + 가이드)
```

---

*ROBOTIS 업스트림 참고: [VR teleoperation guide](https://docs.robotis.com/docs/systems/aiworker/quick_start_guide/operation_guide/vr_teleoperation)*
