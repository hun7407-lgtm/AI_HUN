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

"""``FFW_SG2_MOBILE.usd`` 를 생성한다 — FFW_SG2 의 주행 가능 변형.

    cd /workspace/cyclo_lab
    ./third_party/IsaacLab/isaaclab.sh -p scripts/tools/build_ffw_sg2_mobile_usd.py

스톡 ``FFW_SG2.usd`` 는 정지 상태 매니퓰레이션용으로 저작되어 있어서 주행을 네 군데서
막는다. 이 스크립트는 원본을 **참조**하고 그 네 가지만 override 하는 얇은 레이어를 쓴다
(41 MB 복제 대신 ~2 KB, 원본이 갱신되면 자동으로 따라감).

    1. FixedJoint 가 섀시(``world`` 링크)를 시뮬레이션 월드에 용접
    2. 휠 drive 조인트에 +/-1080 deg 스톱 (3바퀴 -> 1.63 m 만에 정지)
    3. 좌/우 휠 충돌 메시가 꺼져 있음 (뒷바퀴만 지면과 접촉)
    4. 자기충돌 ON — 휠 콜라이더를 켜는 순간 하우징과 겹쳐 로봇이 발사됨

원본 USD 는 절대 수정하지 않는다. 스톡 태스크는 그대로 동작한다.
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="FFW_SG2_MOBILE.usd 생성")
parser.add_argument("--force", action="store_true", help="기존 파일 덮어쓰기")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

args_cli.headless = True
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os

from pxr import Sdf, Usd, UsdPhysics, UsdShade

from cyclo_lab.assets.robots import CYCLO_LAB_ASSETS_DATA_DIR

FFW_DIR = f"{CYCLO_LAB_ASSETS_DATA_DIR}/robots/FFW"
SRC_NAME = "FFW_SG2.usd"
OUT = f"{FFW_DIR}/FFW_SG2_MOBILE.usd"

ROBOT = "/Root/ffw_sg2_follower"
MODULES = ("left", "rear", "right")

# 연속 회전 대용. lower>upper (UsdPhysics 의 "무제한" 관례) 는 PhysX 가 문자 그대로 읽어
# 휠을 ~0 rad 에 잠가버리고, RemoveProperty 는 로컬 오피니언만 지워 참조 레이어의
# +/-1080 이 살아남는다. 그래서 명시적으로 큰 값을 쓴다 (~27 000 회전).
FREE_SPIN_DEG = 1.0e7

WHEEL_STATIC_FRICTION = 1.2
WHEEL_DYNAMIC_FRICTION = 1.0


def info(*args):
    # Kit 이 버퍼를 삼키므로 flush 필수.
    print(*args, flush=True)


def main() -> int:
    if os.path.exists(OUT) and not args_cli.force:
        info(f"[X] 이미 존재함: {OUT}\n    덮어쓰려면 --force")
        return 1

    if os.path.exists(OUT):
        os.remove(OUT)

    info("=" * 88)
    info("FFW_SG2_MOBILE.usd 생성")
    info("=" * 88)
    info(f"  참조 원본 : {FFW_DIR}/{SRC_NAME}  (수정하지 않음)")
    info(f"  출력      : {OUT}")

    stage = Usd.Stage.CreateNew(OUT)
    root = stage.DefinePrim("/Root")
    # 같은 폴더 기준 상대 경로 -> 폴더째 옮겨도 참조가 깨지지 않는다.
    root.GetReferences().AddReference(f"./{SRC_NAME}")
    stage.SetDefaultPrim(root)

    changes = []

    # [1] 베이스 고정 해제 -----------------------------------------------------
    fixed_joint = stage.GetPrimAtPath(f"{ROBOT}/FixedJoint")
    if not fixed_joint.IsValid():
        info(f"[X] FixedJoint 를 찾을 수 없음: {ROBOT}/FixedJoint")
        return 1
    fixed_joint.SetActive(False)
    changes.append("FixedJoint 비활성 (floating base)")
    info("\n[1] FixedJoint 비활성 -> 베이스 고정 해제")

    # [2] 휠 drive limit 해제 --------------------------------------------------
    info(f"\n[2] 휠 drive limit: +/-1080 deg -> +/-{FREE_SPIN_DEG:.0e} deg")
    for module in MODULES:
        joint = stage.GetPrimAtPath(f"{ROBOT}/joints/{module}_wheel_drive")
        if not joint.IsValid():
            info(f"[X] 조인트 없음: {module}_wheel_drive")
            return 1
        for attr_name, value in (
            ("physics:lowerLimit", -FREE_SPIN_DEG),
            ("physics:upperLimit", FREE_SPIN_DEG),
        ):
            attr = joint.GetAttribute(attr_name) or joint.CreateAttribute(
                attr_name, Sdf.ValueTypeNames.Float
            )
            attr.Set(value)
        changes.append(f"{module}_wheel_drive 무제한")
        info(f"    {module}_wheel_drive")

    # [3] 휠 충돌체 활성 -------------------------------------------------------
    info("\n[3] 휠 충돌체 활성 (좌/우가 꺼져 있었음)")
    wheel_colliders = []
    for module in MODULES:
        path = f"{ROBOT}/{module}_wheel_drive_link/collisions/{module}_wheel_drive_link/mesh"
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            info(f"[X] 충돌 prim 없음: {path}")
            return 1
        attr = prim.GetAttribute("physics:collisionEnabled")
        before = attr.Get() if attr else None
        if not attr:
            attr = prim.CreateAttribute("physics:collisionEnabled", Sdf.ValueTypeNames.Bool)
        attr.Set(True)
        wheel_colliders.append(prim)
        changes.append(f"{module} 휠 충돌 ON")
        info(f"    {module:5s} collisionEnabled: {before} -> True")

    # [4] 자기충돌 OFF ---------------------------------------------------------
    articulation = stage.GetPrimAtPath(ROBOT)
    attr = articulation.GetAttribute("physxArticulation:enabledSelfCollisions")
    if not attr:
        info("[X] enabledSelfCollisions 속성 없음")
        return 1
    attr.Set(False)
    changes.append("자기충돌 OFF")
    info("\n[4] enabledSelfCollisions -> False")
    info("    (켜면 휠 콜라이더가 하우징과 겹쳐 로봇이 z=700 m 로 발사됨)")

    # [5] 휠 접지 재질 ---------------------------------------------------------
    info("\n[5] 휠 물리 재질 + 바인딩")
    material = UsdShade.Material.Define(stage, "/Root/wheelPhysicsMaterial")
    UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    material_api = UsdPhysics.MaterialAPI(material.GetPrim())
    material_api.CreateStaticFrictionAttr().Set(WHEEL_STATIC_FRICTION)
    material_api.CreateDynamicFrictionAttr().Set(WHEEL_DYNAMIC_FRICTION)
    material_api.CreateRestitutionAttr().Set(0.0)
    for prim in wheel_colliders:
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            material, UsdShade.Tokens.weakerThanDescendants, "physics"
        )
    changes.append(f"휠 재질 바인딩 x{len(wheel_colliders)}")
    info(f"    static={WHEEL_STATIC_FRICTION} dynamic={WHEEL_DYNAMIC_FRICTION}"
         f" -> 휠 {len(wheel_colliders)}개")

    stage.GetRootLayer().documentation = (
        "FFW_SG2 drivable variant. References FFW_SG2.usd and overrides only what the stock "
        "asset locks down for stationary manipulation: the world FixedJoint, the +/-1080 deg "
        "wheel stops, the disabled left/right wheel colliders, and self-collision. "
        "Self-collision must stay off: the wheel meshes overlap their housings. "
        "Regenerate with scripts/tools/build_ffw_sg2_mobile_usd.py"
    )
    stage.Save()

    info("\n" + "=" * 88)
    info(f"완료 — 변경 {len(changes)}건, {os.path.getsize(OUT) / 1024:.1f} KB (41 MB 원본 참조)")
    info("=" * 88)
    for change in changes:
        info(f"  - {change}")
    info("\n  검증: ./third_party/IsaacLab/isaaclab.sh -p scripts/tools/check_ffw_sg2_mobile.py")
    return 0


if __name__ == "__main__":
    code = main()
    simulation_app.close()
