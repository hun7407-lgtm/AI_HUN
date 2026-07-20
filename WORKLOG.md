# WORKLOG — 데이터 수집 진행 기록

**작성자:** Hun Kim (hun7728@hanyang.ac.kr) · **수정일:** 2026-07-20

VR teleoperation 기반 데이터 수집·후처리 진행 내역.

---

## 1. Task Success Episode 데이터 수집

- **Task**: `Box rear table far (Thin)`
- **결과**: Success Episode **11개** 수집
- 사람이 Meta Quest 3로 로봇을 원격조작하여 태스크 성공 시연을 녹화 (`*_raw.hdf5`).

## 2. 수집 데이터 후처리 및 분석

- 수집된 Demonstration 데이터에 대해 **IK Convert → Annotation** 수행.
  - IK Convert: 관절 녹화 → 엔드이펙터(IK) 액션 변환 (`*_ik.hdf5`)
  - Annotation: 서브태스크 구간(grasp/move/place) 라벨링 (`*_annotate.hdf5`)
- 수집 데이터의 **품질 및 Task 수행 경향**을 확인하기 위한 **기초 데이터 분석** 진행.

## 3. VR ↔ Isaac Sim Connection 안정화 루틴 구성

VR 환경과 Isaac Sim 간 연결 안정화를 위해 **반복 수행 가능한 연결 절차**를 정리함.
(전체 체크리스트는 [`README.md` §4-1](README.md) 참고.)

**주요 연결 절차:**
1. USB를 통해 PC와 VR Headset 연결
2. Connection Script 실행
3. VR Controller — Dashboard Button 입력
4. PC에서 Local Host 실행 여부 확인
5. Meta Quest Browser에서 Local Host 접속
6. Dashboard의 [Launch Record] Button 실행
7. Isaac Sim 실행 후 VR Browser의 Local Host 연결 상태 재확인
8. Isaac Sim 내에서 Keyboard `B` 입력 후 Controller Grip 약 3초간 입력

- **연결 실패 시**: `R` 입력 → 다시 `B` 입력 및 Controller Grip 입력 절차 반복
- **Task 수행 후 결과 확인**
  - 만족: `N` 입력으로 데이터 저장
  - 실패/불만족: `R` 입력 후 재수행
