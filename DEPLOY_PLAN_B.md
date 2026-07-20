# Plan B 배포 & 테스트 절차 — 모바일 베이스 주행 녹화

**작성자:** Hun Kim (hun7728@hanyang.ac.kr) · **수정일:** 2026-07-20

VR 조이스틱으로 **베이스를 직접 몰면서** L-table 데이터를 녹화하고, 베이스 속도까지 기록(22-dim)하는 흐름의 배포 절차. 코드는 전부 오버레이에 있고, 아래는 **컨테이너/실행에 반영**하는 단계다.

> 기존 stock 녹화(teleport, 19-dim)는 **그대로 동작**한다. 아래는 새 **Mobile 태스크**를 쓰기 위한 추가 절차.

---

## 0. 코드 반영 상태 (이미 완료)

| 변경 | 위치 | 반영 |
|---|---|---|
| A/B 회전 버튼 | `overlays/robotis_applications/.../vr_publisher_sg2.py` | ✅ 라이브·컨테이너 배포됨 |
| Mobile 태스크 + cmd_vel 구동 | `overlays/cyclo_lab/...` | ✅ `sync_overlay.sh`로 반영됨 |
| 대시보드(세션네이밍 + Mobile 선택) | `overlays/cyclo_lab/sg2_ltable_dashboard.py` | ✅ 반영됨 |

**남은 건 "재시작"뿐이다** (실행 중인 프로세스가 옛 코드를 메모리에 들고 있으므로).

---

## 1. VR 퍼블리셔 재시작 (A/B 버튼 로드)

`robotis_vuer`는 **symlink-install**(egg-link)이라 소스 수정이 즉시 반영된다 — **colcon rebuild 불필요**, 노드만 재시작하면 된다.

- **가장 쉬운 방법**: 대시보드에서 **Launch VR + Controller**(또는 Launch Record)를 다시 누르면 VR 노드가 새 코드로 재기동된다.
- 수동으로 확인하려면 컨테이너에서 VR publisher 프로세스를 죽였다가 대시보드로 재실행.

> 코드가 반영되면 컨테이너 로그에 `[BTN_DEBUG] ...` 와 버튼 상태가 찍힌다.

---

## 2. 대시보드 재시작 (세션 네이밍 + Mobile 태스크 로드)

현재 실행 중인 대시보드는 **재시작 전 옛 버전**을 메모리에 들고 있다. 새 코드를 쓰려면 재시작:

```bash
# 실행 중인 대시보드 종료 후 재실행
cd ~/AIWORKER/cyclo_lab
python3 sg2_ltable_dashboard.py
# http://localhost:8765
```

재시작하면:
- 태스크 목록에 **"L-Table Pick & Place (mobile base)"** 가 뜬다.
- 데이터셋이 세션 네이밍으로 저장된다: `datasets/{base}/{YYYYMMDD}/{HHMM}_{ip}_{src}_{base}_{stage}.hdf5`

---

## 3. 모바일 베이스로 녹화

1. 대시보드 → 태스크 **"L-Table Pick & Place (mobile base)"** 선택
2. **Launch Record** → Isaac Sim GUI가 뜸 (로봇이 **바퀴 위에 서 있음** = 자유 베이스)
3. VR 연결 (USB/ADB 또는 WiFi)
4. **베이스 조작 모드 진입**: 양쪽 **썸스틱 클릭**으로 `LIFT+HEAD` ↔ **`LIFT+CMD_VEL`** 토글 → 컨테이너 로그에 `Mode switched to: LIFT+CMD_VEL` 확인
5. 조작:
   | 입력 | 동작 |
   |---|---|
   | **왼쪽 썸스틱** | 베이스 이동 (전후 = x, 좌우 = y) |
   | **A 버튼 (오른손, 홀드)** | 우회전 |
   | **B 버튼 (오른손, 홀드)** | 좌회전 |
   | 양쪽 그립 3초 | 팔 teleop 활성 (기존과 동일) |
6. 박스 집고 → **베이스를 몰아** L-table로 이동 → 놓기 (teleport 아님, 실제 주행)
7. **N** 저장 / **R** 폐기 (기존과 동일)

> 저장 파일: `datasets/ffw_sg2_l_table_mobile/{날짜}/{시간}_..._raw.hdf5`

---

## 4. LeRobot 변환 (자동 22-dim)

변환기는 `obs/base_velocity` 존재를 **자동 감지**해서 22-dim으로 변환한다 (없으면 19-dim). 별도 플래그 불필요:

```bash
docker exec -it cyclo_lab bash -lc '
cd /workspace/cyclo_lab
lerobot-python scripts/sim2real/imitation_learning/data_converter/isaaclab2lerobot.py \
  --task Cyclo-Real-Pick-Place-LTable-Mobile-FFW-SG2-v0 \
  --robot_type FFW_SG2 --fps 15 \
  --dataset_file ./datasets/ffw_sg2_l_table_mobile/<날짜>/<...>_joint.hdf5
'
```

로그에 `Base velocity: present -> 22-dim state/action` 이 뜨면 정상. 카메라 4개도 `observation.images.rgb.cam_*`로 나온다 → **실제 로봇 포맷과 일치**.

---

## 5. 검증 체크리스트

| 확인 | 방법 | 기대 |
|---|---|---|
| 베이스가 실제로 굴러가나 | GUI에서 썸스틱/버튼 조작 | 바퀴 회전 + 이동 (제자리 텔레포트 아님) |
| A/B 회전 방향 | A=우, B=좌 | 맞으면 OK, 반대면 `button_turn_rate` 부호 조정 |
| 팔이 몸통 통과 안 하나 | 팔을 몸쪽으로 | 자기충돌로 막힘 |
| raw에 base_velocity 기록 | `obs/base_velocity` shape (N,3) | 존재 |
| 변환 후 22-dim | 변환 로그 / info.json | state/action shape 22 |

---

## ⚠️ 알려진 제약 / 미완성

- **Mimic/datagen은 아직 mobile용으로 안 됨** — Mobile 녹화 raw는 22-dim이고 base velocity가 있어서, IK convert/annotate/datagen 파이프라인이 이를 처리하려면 추가 작업 필요. **현재는 녹화까지만** 지원.
- **카메라 외부 파라미터는 placeholder** — head baseline(0.06m 추정), wrist는 d405 실측이지만 광학축은 검증 필요. 실제 로봇과 정밀 대조 전이면 튜닝 권장.
- **자기충돌 ON이지만 바퀴는 필터링** — 바퀴는 바닥만 충돌(로봇 몸체와는 충돌 안 함). 정상 설계.
- **VR 실측 미완** — cmd_vel이 실제로 베이스를 굴리는지, 조작감은 헤드셋으로 확인 필요.

---

## 되돌리기 (문제 시)

- Mobile 태스크만 안 쓰면 됨 — 기존 stock 태스크/녹화는 무손상.
- 세션 네이밍 끄기: 대시보드 `USE_SESSION_NAMING = False`.
- 퍼블리셔 A/B 롤백: `button_turn_rate` 관련 블록 제거 후 VR 재시작 (썸스틱 회전으로 복귀).
