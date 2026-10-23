# Script 3: Move SO101 through a 3-step sequence
# Simulates: home -> reach forward -> return home

import numpy as np
import asyncio
from isaacsim.core.api.world import World
from isaacsim.core.prims import SingleArticulation as Articulation
from isaacsim.core.utils.types import ArticulationAction

world = World.instance()
if world is None:
    world = World()

robot = Articulation(prim_path="/so101_new_calib")
robot.initialize()

print("Joint order:")
for i, name in enumerate(robot.dof_names):
    print(i, name)

# Define 3 waypoints in degrees
HOME  = np.array([  0.0,   0.0,   0.0,  0.0,  0.0,  0.0])
REACH = np.array([ 20.0,  30.0, -40.0,  0.0,  0.0,  0.0])
GRASP = np.array([ 20.0,  15.0, -20.0, 20.0,  0.0, 30.0])

WAYPOINTS = [HOME, REACH, GRASP, HOME]
LABELS = ["Home", "Reach forward", "Pre-grasp pose", "Return home"]
HOLD_STEPS = 100  # number of simulation steps to hold each waypoint

HOLD_STEPS = 100

async def run_sequence():
    joint_indices = np.arange(len(robot.dof_names))

    for label, waypoint_deg in zip(LABELS, WAYPOINTS):
        print(f"Moving to: {label}")

        action = ArticulationAction(
            joint_positions=np.radians(waypoint_deg),
            joint_indices=joint_indices
        )

        robot.apply_action(action)

        for _ in range(HOLD_STEPS):
            await world.step_async()s

    print("Sequence complete.")

# Run the async sequence
asyncio.ensure_future(run_sequence())