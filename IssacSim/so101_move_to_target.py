# Script 2: Move SO101 to a target joint configuration
# Adjust the target_positions array to match your arm's joint order

import numpy as np
from isaacsim.core.api.world import World
from isaacsim.core.prims import SingleArticulation as Articulation
from isaacsim.core.utils.types import ArticulationAction



world = World.instance()
if world is None:
    world = World()

robot = Articulation(prim_path="/so101_new_calib")
robot.initialize()

joint_names = robot.dof_names
print("Joint order:")
for i, name in enumerate(joint_names):
    print(i, name)

# Target joint angles in DEGREES (we convert to radians below)
# Order: [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper_left, gripper_right]
# Start small! Use safe angles to avoid hitting joint limits.
target_deg = np.array([
    30.0,   # shoulder_pan:  rotate base 30 degrees
    20.0,   # shoulder_lift: tilt upper arm up 20 degrees
   -30.0,   # elbow_flex:    bend elbow 30 degrees
     0.0,   # wrist_flex:    keep wrist straight
     0.0,   # wrist_roll:    keep wrist level
    10.0,   # gripper:  open gripper slightly
])

target_rad = np.radians(target_deg)  # convert to radians


# Apply the target position to the joint drives
action = ArticulationAction(
    joint_positions=target_rad,
    joint_indices=np.arange(len(target_rad))
)

robot.apply_action(action)

print('Target command sent!')
print('Arm will move to target on the next simulation steps.')
print('Watch the viewport.')
