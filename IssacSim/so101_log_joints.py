# Script 4: Move arm and log joint data to CSV
import numpy as np
import asyncio
import csv
import os
import omni.timeline
import omni.kit.app

from isaacsim.core.api.world import World
from isaacsim.core.prims import SingleArticulation as Articulation
from isaacsim.core.utils.types import ArticulationAction

world = World.instance()
if world is None:
    world = World()

robot = Articulation(prim_path="/so101_new_calib")

timeline = omni.timeline.get_timeline_interface()
timeline.play()

# Your SO101 has 6 controllable joints, not 7
HOME  = np.radians([ 0.0,  0.0,  0.0, 0.0, 0.0, 0.0])
REACH = np.radians([20.0, 30.0,-40.0, 0.0, 0.0, 0.0])

base_dir = r"C:\Users\OMNI-User\Desktop\shaoyu\SO-ARM100\Simulation\SO101\0504-test"
LOG_PATH = os.path.join(base_dir, "so101_joint_log.csv")

async def move_and_log():
    # Let Isaac Sim initialize physics first
    for _ in range(10):
        await omni.kit.app.get_app().next_update_async()

    robot.initialize()

    joint_indices = np.arange(len(robot.dof_names))

    with open(LOG_PATH, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["step", "target_label"] + robot.dof_names)

        global_step = 0

        for target, label in [(HOME, "home"), (REACH, "reach")]:
            print(f"Moving to {label}...")

            action = ArticulationAction(
                joint_positions=target,
                joint_indices=joint_indices
            )

            for step in range(150):
                robot.apply_action(action)

                await omni.kit.app.get_app().next_update_async()

                if step % 10 == 0:
                    pos = robot.get_joint_positions().tolist()
                    writer.writerow([global_step, label] + pos)
                    f.flush()

                global_step += 1

    print(f"Done. Joint data saved to: {LOG_PATH}")
    print("Open the CSV in a spreadsheet to plot joint trajectories.")

asyncio.ensure_future(move_and_log())