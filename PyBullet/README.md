# LeRobot SO-ARM101 PyBullet Simulation

A physics-based simulation environment for the **LeRobot SO-ARM101** robotic arm, built with [PyBullet](https://pybullet.org/). This project lets you load, visualize, and interactively control the SO-ARM101 in a simulated 3D environment — no physical hardware required.

---

## What This Project Does

- Loads the SO-ARM101 robot arm into a PyBullet physics simulation
- Displays joint names, types, and range-of-motion limits
- Lets users specify target joint angles or end-effector positions and watch the arm move in real time
- Supports both **GUI mode** (live 3D viewer) and **DIRECT/headless mode** (for scripting and notebooks)
- Demonstrates inverse kinematics (IK) so you can command the arm by specifying where the gripper should go, not just raw joint angles
- Includes a simple pick-and-place demo: the arm grasps a small box sitting on a table

---

## Prerequisites

| Requirement | Version |
|---|---|
| Python | 3.8 – 3.11 |
| pip | latest recommended |
| Git | any recent version |

> **Windows users:** PyBullet works best inside WSL2 or a Conda environment. The GUI renderer requires a working OpenGL driver.

---

## Installation

### 1 — Clone the repository

```bash
git clone https://github.com/<your-username>/so-arm101-pybullet-sim.git
cd so-arm101-pybullet-sim
```

### 2 — Create a virtual environment (recommended)

```bash
python -m venv .venv
# Linux / macOS
source .venv/bin/activate
# Windows
.venv\Scripts\activate
```

### 3 — Install dependencies

```bash
pip install -r requirements.txt
```

**Or install manually:**

```bash
pip install pybullet numpy jupyter
```

### 4 — Verify PyBullet

```bash
python -c "import pybullet; print(pybullet.getPhysicsEngineParameters)"
```

You should see a dictionary of physics engine parameters printed — no errors means you're good to go.

---

## Repository Structure

```
so-arm101-pybullet-sim/
├── urdf/
│   └── so_arm101.urdf          # Robot description file (URDF)
├── notebooks/
│   └── 02_pybullet_basics.ipynb  # Step-by-step tutorial notebook
├── sim/
│   ├── __init__.py
│   ├── robot.py                # Robot wrapper class
│   └── controller.py           # Joint & IK controllers
├── demos/
│   └── pick_and_place.py       # Pick-and-place demo script
├── requirements.txt
└── README.md
```

---

## Quick Start

### Run the interactive GUI simulation

```bash
python -m sim.robot --gui
```

The PyBullet window opens with the SO-ARM101 loaded on a flat plane. Use the sliders on the right panel to control individual joint angles in real time.

### Run the pick-and-place demo (headless)

```bash
python demos/pick_and_place.py
```

This runs the full pick-and-place sequence without opening a window and prints joint states and IK results to the terminal.

### Open the tutorial notebook

```bash
jupyter notebook notebooks/02_pybullet_basics.ipynb
```

The notebook walks through five progressive exercises:

| Cell | Topic |
|---|---|
| 1 | Connect to PyBullet, load a plane and the Kuka arm |
| 2 | Inspect joint names, types, and limits |
| 3 | Set joint positions, render a camera image |
| 4 | Compute inverse kinematics and move the arm |
| 5 | Pick-and-place simulation with a small box |

---

## How to Control the Arm

### Option A — Joint-angle control (direct)

Specify target angles (in radians) for each joint:

```python
import pybullet as p
import pybullet_data, time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
robot_id = p.loadURDF("urdf/so_arm101.urdf", useFixedBase=True)

# Move joint 0 to 45°, joint 1 to -30°, joint 2 to 60°
target_angles = [0.785, -0.524, 1.047, 0, 0, 0]
num_joints = p.getNumJoints(robot_id)

for i, angle in enumerate(target_angles):
    p.setJointMotorControl2(
        robot_id, i,
        controlMode=p.POSITION_CONTROL,
        targetPosition=angle
    )

for _ in range(500):
    p.stepSimulation()
    time.sleep(1 / 240)
```

### Option B — End-effector target (inverse kinematics)

Tell the arm where the gripper should go in 3D space:

```python
target_position = [0.3, 0.1, 0.25]   # x, y, z in metres
end_effector_link = 5                  # link index of the gripper

joint_angles = p.calculateInverseKinematics(robot_id, end_effector_link, target_position)

for i, angle in enumerate(joint_angles):
    p.setJointMotorControl2(
        robot_id, i,
        controlMode=p.POSITION_CONTROL,
        targetPosition=angle
    )

for _ in range(500):
    p.stepSimulation()
    time.sleep(1 / 240)
```

---

## Simulation Modes

| Mode | Flag | Use Case |
|---|---|---|
| GUI (live viewer) | `p.connect(p.GUI)` | Interactive control, demos |
| DIRECT (headless) | `p.connect(p.DIRECT)` | Scripting, notebooks, CI |

Switch between modes by changing the `connect` call — all other code stays the same.

---

## URDF Notes

The `urdf/so_arm101.urdf` file describes the SO-ARM101's kinematic chain, link geometries, inertial properties, and joint limits. If you have access to the official LeRobot URDF, place it in the `urdf/` folder. Otherwise, a simplified placeholder URDF is included to get you started.

For the official URDF and CAD files, see the [LeRobot hardware repository](https://github.com/huggingface/lerobot).

---

## Troubleshooting

**`ImportError: No module named 'pybullet'`**
→ Make sure your virtual environment is activated and run `pip install pybullet`.

**GUI window doesn't open on a remote server**
→ Use `p.connect(p.DIRECT)` for headless mode, or set up a virtual display with `Xvfb`.

**URDF not found**
→ Confirm the `urdf/so_arm101.urdf` path is correct, or update the `loadURDF` call to point to your file.

**Simulation runs too fast / too slow**
→ Adjust the `time.sleep(1 / 240)` value. PyBullet's default timestep is 1/240 s.

---

## Contributing

Pull requests are welcome! Please open an issue first to discuss any major changes. When submitting code, make sure all notebook cells run cleanly in headless (`DIRECT`) mode.

---

## License

This project is licensed under the **MIT License**. See [LICENSE](LICENSE) for details.

---

## Acknowledgements

- [PyBullet](https://github.com/bulletphysics/bullet3) — physics simulation engine
- [HuggingFace LeRobot](https://github.com/huggingface/lerobot) — SO-ARM101 hardware design and software stack
- [pybullet_data](https://github.com/bulletphysics/bullet3/tree/master/data) — built-in URDFs and assets used for testing
