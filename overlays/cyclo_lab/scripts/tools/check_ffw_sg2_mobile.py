# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""``FFW_SG2_MOBILE`` 회귀 검증: 서는가 / 굴러가는가 / 홀로노믹인가.

    cd /workspace/cyclo_lab
    ./third_party/IsaacLab/isaaclab.sh -p scripts/tools/check_ffw_sg2_mobile.py --headless
    ./third_party/IsaacLab/isaaclab.sh -p scripts/tools/check_ffw_sg2_mobile.py   # 눈으로

스톡 FFW_SG2 로 돌리면 전부 실패하는 것이 정상이다 (베이스가 월드에 용접되어 있음).
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="FFW_SG2_MOBILE 회귀 검증")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import math
import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.sim import SimulationContext
from isaaclab.utils.math import euler_xyz_from_quat

from cyclo_lab.assets.robots.FFW_SG2 import SG2_SWERVE_WHEEL_RADIUS
from cyclo_lab.assets.robots.FFW_SG2_MOBILE import FFW_SG2_MOBILE_CFG
from cyclo_lab.controllers import SwerveController

DT = 1.0 / 120.0
# 스톡 휠 스톱: +/-1080 deg = 18.85 rad = 1.63 m 주행 후 정지. 이 선을 넘어야 개조 성공.
STOCK_LIMIT_RAD = 1080.0 * math.pi / 180.0
STOCK_LIMIT_M = STOCK_LIMIT_RAD * SG2_SWERVE_WHEEL_RADIUS

results: list[tuple[str, bool, str]] = []


def info(*args):
    print(*args, flush=True)  # Kit 이 버퍼를 삼키므로 flush 필수.


def record(name: str, passed: bool, detail: str):
    results.append((name, passed, detail))
    info(f"    {'O 통과' if passed else 'X 실패'}  {detail}")


def main() -> int:
    sim = SimulationContext(sim_utils.SimulationCfg(dt=DT, device=args_cli.device))
    sim.set_camera_view(eye=(4.0, 4.0, 3.0), target=(0.0, 0.0, 0.5))
    sim_utils.GroundPlaneCfg().func("/World/ground", sim_utils.GroundPlaneCfg())
    sim_utils.DomeLightCfg(intensity=2500.0).func(
        "/World/light", sim_utils.DomeLightCfg(intensity=2500.0)
    )

    robot = Articulation(FFW_SG2_MOBILE_CFG.replace(prim_path="/World/Robot"))
    sim.reset()
    swerve = SwerveController(robot)

    info("=" * 88)
    info("FFW_SG2_MOBILE 회귀 검증")
    info("=" * 88)
    info(f"  bodies={robot.num_bodies} joints={robot.num_joints}")
    info(f"  모듈 순서: {swerve.module_keys}")

    def step(n: int, twist=None):
        for _ in range(n):
            if twist is None:
                swerve.stop()
            else:
                swerve.apply(*twist)
            robot.write_data_to_sim()
            sim.step()
            robot.update(DT)

    # 착지한 실제 자세를 기억해 두고 여기로 복귀한다.
    # default_root_state 를 쓰면 안 된다: 그것은 init_state.pos (0, 0, 0.01) 인데
    # 루트 바디의 실제 높이는 1.4396 이다 (USD 내부 오프셋 1.43 m). 그대로 써넣으면
    # 로봇을 1.43 m 땅속에 파묻어 바퀴가 헛돌게 된다.
    home_pose: list[torch.Tensor] = []

    def reset_pose():
        robot.write_root_pose_to_sim(home_pose[0].clone())
        robot.write_root_velocity_to_sim(torch.zeros((1, 6), device=robot.device))
        step(180)  # 다시 안정될 때까지

    def align_steer(twist):
        """구동 전에 조향만 정렬한다. 안 하면 조향이 도는 중에 바퀴가 밀어 로봇이 휘청인다."""
        angles, _ = swerve.compute(*twist)
        for _ in range(180):
            robot.set_joint_position_target(angles, joint_ids=swerve._steer_ids)
            robot.set_joint_velocity_target(
                torch.zeros((1, len(swerve._drive_ids)), device=robot.device),
                joint_ids=swerve._drive_ids,
            )
            robot.write_data_to_sim()
            sim.step()
            robot.update(DT)

    def yaw() -> float:
        _, _, y = euler_xyz_from_quat(robot.data.root_quat_w)
        return y[0].item()

    # [1] 베이스가 자유로운가 -------------------------------------------------
    info("\n[1] 베이스 고정 해제")
    record("고정 해제", not robot.is_fixed_base, f"is_fixed_base={robot.is_fixed_base} (False 여야 함)")

    # [2] 바퀴로 서는가 -------------------------------------------------------
    info("\n[2] 중력하 착지 안정성 (3초)")
    step(360)
    z = robot.data.root_pos_w[0, 2].item()
    speed = torch.norm(robot.data.root_lin_vel_w[0]).item()
    record("착지", speed < 0.2 and 0.5 < z < 2.5, f"root_z={z:.4f} 잔류속도={speed:.3f} m/s")

    # 착지한 자세를 기준점으로 저장 (reset_pose 가 여기로 되돌린다).
    home_pose.append(torch.cat([robot.data.root_pos_w[:, :3], robot.data.root_quat_w], dim=-1).clone())

    # [3] 스톡 휠 스톱을 넘어 주행하는가 --------------------------------------
    info(f"\n[3] 직진 주행 10초 — 스톡 한계 {STOCK_LIMIT_M:.2f} m 돌파 여부")
    reset_pose()
    align_steer((0.865, 0.0, 0.0))
    start = robot.data.root_pos_w[0, :3].clone()
    angle_start = robot.data.joint_pos[0, swerve._drive_ids].clone()
    step(1200, twist=(0.865, 0.0, 0.0))  # 10 rad/s 상당
    moved = torch.norm((robot.data.root_pos_w[0, :3] - start)[:2]).item()
    spun = (robot.data.joint_pos[0, swerve._drive_ids] - angle_start).mean().item()
    record(
        "직진 주행",
        moved > 2.0 and abs(spun) > STOCK_LIMIT_RAD,
        f"{moved:.2f} m 이동, 휠 {spun:.1f} rad ({spun * 57.2958:.0f} deg) 회전",
    )

    # [4] 홀로노믹 기동 -------------------------------------------------------
    # 기동마다 자세를 리셋한다. 스핀이 누적되면 로봇 기준 지령과 월드 기준 측정이 어긋나
    # 정상 동작을 실패로 오판한다.
    info("\n[4] 홀로노믹 기동 (기동마다 자세 리셋)")

    info("\n  [4-1] 게걸음 vy=+0.5 (기대: +y 로 이동, 회전 없음)")
    reset_pose()
    align_steer((0.0, 0.5, 0.0))
    start = robot.data.root_pos_w[0, :3].clone()
    y0 = yaw()
    step(360, twist=(0.0, 0.5, 0.0))
    d = robot.data.root_pos_w[0, :3] - start
    dyaw = math.degrees(yaw() - y0)
    record(
        "게걸음",
        d[1].item() > 1.0 and abs(d[0].item()) < 0.3,
        f"dx={d[0].item():+.3f} dy={d[1].item():+.3f} dyaw={dyaw:+.1f} deg",
    )

    info("\n  [4-2] 제자리 회전 omega=+0.8 (기대: 회전만, 이동 없음)")
    reset_pose()
    align_steer((0.0, 0.0, 0.8))
    start = robot.data.root_pos_w[0, :3].clone()
    y0 = yaw()
    step(360, twist=(0.0, 0.0, 0.8))
    d = robot.data.root_pos_w[0, :3] - start
    dyaw = math.degrees(yaw() - y0)
    drift = math.hypot(d[0].item(), d[1].item())
    record("제자리 회전", abs(dyaw) > 60.0 and drift < 0.4, f"dyaw={dyaw:+.0f} deg 이동={drift:.3f} m")

    info("\n  [4-3] 대각선 vx=vy=+0.35 (기대: +x +y 동시)")
    reset_pose()
    align_steer((0.35, 0.35, 0.0))
    start = robot.data.root_pos_w[0, :3].clone()
    step(360, twist=(0.35, 0.35, 0.0))
    d = robot.data.root_pos_w[0, :3] - start
    record(
        "대각선",
        d[0].item() > 0.7 and d[1].item() > 0.7,
        f"dx={d[0].item():+.3f} dy={d[1].item():+.3f}",
    )

    # 요약 -------------------------------------------------------------------
    info("\n" + "=" * 88)
    passed = sum(1 for _, ok, _ in results if ok)
    info(f"결과: {passed}/{len(results)} 통과")
    info("=" * 88)
    for name, ok, detail in results:
        info(f"  {'O' if ok else 'X'}  {name:12s} {detail}")
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    code = main()
    simulation_app.close()
