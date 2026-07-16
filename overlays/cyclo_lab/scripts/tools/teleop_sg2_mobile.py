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

"""키보드로 FFW_SG2 모바일 베이스를 조종한다 (3륜 홀로노믹 스워브).

    cd /workspace/cyclo_lab
    ./third_party/IsaacLab/isaaclab.sh -p scripts/tools/teleop_sg2_mobile.py

Isaac Sim 창을 클릭해서 포커스를 준 뒤 키를 누른다.

    ↑ / Numpad 8   전진              Z / Numpad 7   좌회전
    ↓ / Numpad 2   후진              X / Numpad 9   우회전
    ← / Numpad 6   좌 게걸음          L              정지
    → / Numpad 4   우 게걸음

    E  속도 +25%       Q  속도 -25%       R  로봇위치 리셋
    C  로봇기준 / 월드기준 전환

전진과 게걸음을 동시에 누르면 대각선으로 간다 (홀로노믹).
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="FFW_SG2 모바일 베이스 키보드 조종")
parser.add_argument("--speed", type=float, default=0.5, help="초기 속도 배율 (기본 0.5)")
parser.add_argument("--world_frame", action="store_true", help="월드 기준으로 시작")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# 조종하려면 창이 필요하다.
args_cli.headless = False
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.devices import Se2Keyboard, Se2KeyboardCfg
from isaaclab.sim import SimulationContext

from cyclo_lab.assets.robots.FFW_SG2_MOBILE import FFW_SG2_MOBILE_CFG, SG2_MOBILE_SPAWN_HEIGHT
from cyclo_lab.controllers import SwerveController

DT = 1.0 / 120.0


class Teleop:
    """키보드 상태 -> 스워브 지령."""

    def __init__(self, robot, speed: float, world_frame: bool):
        self.robot = robot
        self.swerve = SwerveController(robot)
        self.speed = speed
        self.world_frame = world_frame

        # sensitivity 는 1.0 으로 두고 배율은 여기서 곱한다 (런타임에 바꿔야 하므로).
        self.keyboard = Se2Keyboard(
            Se2KeyboardCfg(v_x_sensitivity=1.0, v_y_sensitivity=1.0, omega_z_sensitivity=1.5)
        )
        self.keyboard.add_callback("E", self._faster)
        self.keyboard.add_callback("Q", self._slower)
        self.keyboard.add_callback("C", self._toggle_frame)
        self.keyboard.add_callback("R", self._reset_pose)
        self._banner()

    def _banner(self):
        print("\n" + "=" * 68, flush=True)
        print(self.keyboard, flush=True)
        print("\t----------------------------------------------", flush=True)
        print("\tSpeed up: E   Speed down: Q", flush=True)
        print("\tToggle robot/world frame: C   Reset pose: R", flush=True)
        print("=" * 68, flush=True)
        self._status()

    def _status(self):
        frame = "월드 기준" if self.world_frame else "로봇 기준"
        print(
            f"  [속도 x{self.speed:.2f}]  전진 최대 {self.speed:.2f} m/s  |  {frame}",
            flush=True,
        )

    def _faster(self):
        self.speed = min(self.speed * 1.25, 4.0)
        self._status()

    def _slower(self):
        self.speed = max(self.speed / 1.25, 0.05)
        self._status()

    def _toggle_frame(self):
        self.world_frame = not self.world_frame
        self._status()

    def set_home(self):
        """착지한 자세를 리셋 기준점으로 저장한다.

        default_root_state 를 쓰면 안 된다: 그것은 init_state.pos (0, 0, 0.01) 인데
        루트 바디의 실제 높이는 1.44 다 (USD 내부 오프셋 1.43 m). 그대로 써넣으면
        로봇을 땅속에 파묻어 바퀴가 헛돈다.
        """
        self._home = torch.cat(
            [self.robot.data.root_pos_w[:, :3], self.robot.data.root_quat_w], dim=-1
        ).clone()

    def _reset_pose(self):
        self.robot.write_root_pose_to_sim(self._home.clone())
        self.robot.write_root_velocity_to_sim(torch.zeros((1, 6), device=self.robot.device))
        print("  [리셋] 로봇을 시작 위치로", flush=True)

    def step(self):
        cmd = self.keyboard.advance()
        vx = float(cmd[0]) * self.speed
        vy = float(cmd[1]) * self.speed
        omega = float(cmd[2]) * self.speed

        if abs(vx) < 1e-4 and abs(vy) < 1e-4 and abs(omega) < 1e-4:
            self.swerve.stop()
        elif self.world_frame:
            self.swerve.apply_world(vx, vy, omega)
        else:
            self.swerve.apply(vx, vy, omega)


def main():
    sim = SimulationContext(sim_utils.SimulationCfg(dt=DT, device=args_cli.device))
    sim.set_camera_view(eye=(3.5, 3.5, 2.5), target=(0.0, 0.0, 0.5))

    sim_utils.GroundPlaneCfg().func("/World/ground", sim_utils.GroundPlaneCfg())
    sim_utils.DomeLightCfg(intensity=2500.0).func("/World/light", sim_utils.DomeLightCfg(intensity=2500.0))

    robot = Articulation(FFW_SG2_MOBILE_CFG.replace(prim_path="/World/Robot"))
    sim.reset()

    teleop = Teleop(robot, speed=args_cli.speed, world_frame=args_cli.world_frame)

    # 바퀴 위에 내려앉을 때까지 잠깐 (스폰 높이 SG2_MOBILE_SPAWN_HEIGHT 만큼의 낙하).
    for _ in range(120):
        robot.write_data_to_sim()
        sim.step()
        robot.update(DT)
    teleop.set_home()  # R 키가 여기로 되돌린다.
    print(f"  착지 완료 (root_z={robot.data.root_pos_w[0, 2].item():.4f}) — 조종 시작\n", flush=True)

    while simulation_app.is_running():
        teleop.step()
        robot.write_data_to_sim()
        sim.step()
        robot.update(DT)


if __name__ == "__main__":
    main()
    simulation_app.close()
