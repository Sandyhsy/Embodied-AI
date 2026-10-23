import numpy as np
import omni.usd
from isaacsim.core.api.world import World
from isaacsim.core.prims import SingleArticulation as Articulation

prim_path = "/so101_new_calib"

stage = omni.usd.get_context().get_stage()
prim = stage.GetPrimAtPath(prim_path)

if not prim.IsValid():
    raise RuntimeError(f"Prim not found: {prim_path}")

world = World.instance() if World.instance() else World()

robot = Articulation(prim_path=prim_path)

world.reset()
robot.initialize()

joint_positions = robot.get_joint_positions()
joint_names = robot.dof_names

print("=== SO101 Joint States ===")
for name, pos in zip(joint_names, joint_positions):
    print(f"{name:25s}: {pos:7.4f} rad ({np.degrees(pos):7.2f} deg)")