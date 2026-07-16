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

"""Drivable (floating-base) variant of FFW_SG2.

The stock ``FFW_SG2.usd`` is authored for stationary manipulation:

* a ``FixedJoint`` welds the chassis root link (``world``) to the simulation world,
* the wheel *drive* joints carry a +/-1080 deg limit (~3 revolutions -> ~1.63 m of travel),
* the left/right wheel collision meshes are disabled (only the rear wheel collides),
* gravity is disabled on the articulation.

Base motion in the stock tasks is therefore a kinematic root teleport
(``write_root_pose_to_sim``), not wheel physics.

Those four locks are lifted in ``FFW_SG2_MOBILE.usd``, a small override layer that
references the stock asset (see ``data/robots/FFW/``). This module only supplies the
matching articulation config; the stock USD is never modified.

Measured on the L-table ground plane: settles at root_z=1.4054 and drives 8.25 m in 10 s
at 96% of the commanded 0.865 m/s. Swerve holonomic motion (crab, spin-in-place) verified.
"""

from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.sim import ArticulationRootPropertiesCfg, RigidBodyPropertiesCfg

from cyclo_lab.assets.robots import CYCLO_LAB_ASSETS_DATA_DIR
from cyclo_lab.assets.robots.FFW_SG2 import FFW_SG2_CFG

# Measured: the contact patch lands at (spawn z - 0.002), so a millimetre of clearance is
# all that is needed for the robot to settle onto its wheels.
SG2_MOBILE_SPAWN_HEIGHT = 0.01

FFW_SG2_MOBILE_CFG: ArticulationCfg = FFW_SG2_CFG.replace(
    spawn=FFW_SG2_CFG.spawn.replace(
        # Override layer over FFW_SG2.usd: floating base, free drive joints, live wheel
        # colliders, wheel contact material. Keeps the stock gripper-friction spawn hook.
        usd_path=f"{CYCLO_LAB_ASSETS_DATA_DIR}/robots/FFW/FFW_SG2_MOBILE.usd",
        rigid_props=RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        # KNOWN LIMITATION: self-collision must stay off. The wheel collision meshes the
        # override layer re-enables overlap their own housings on frame 1, and PhysX answers
        # that penetration by launching the robot to z=700 m. Filtering the wheel<->housing
        # and wheel<->chassis pairs was measured to be insufficient. Cost of leaving it off:
        # the arms can pass through the torso.
        articulation_props=ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=32,
            solver_velocity_iteration_count=1,
            fix_root_link=False,
        ),
    ),
    init_state=FFW_SG2_CFG.init_state.replace(
        pos=(0.0, 0.0, SG2_MOBILE_SPAWN_HEIGHT),
    ),
)
"""FFW_SG2 with a floating base, gravity, live wheel collisions and free drive joints."""
