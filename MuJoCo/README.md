# SO-ARM101 MuJoCo Simulation

A CPU-only physics simulation of the **SO-ARM101** 6-DOF robot arm using [MuJoCo](https://mujoco.org/). This project lets you explore the robot's kinematics through a Jupyter notebook, interact with it using keyboard controls in a live 3D viewer, and inspect the full MJCF model.

---

## What Does This Project Do?

This simulation provides three things:

1. **Jupyter Notebook (`01_mujoco_basics.ipynb`)** — A guided walkthrough of MuJoCo fundamentals, from loading a model to computing forward kinematics and visualising the arm's reachable workspace.

2. **Keyboard-Controlled Viewer (`so_arm101_keyboard_sim.py`)** — A live interactive window where you can drive each joint of the SO-ARM101 with keyboard keys, load preset poses, toggle physics, and print the TCP position in real time.

3. **Standalone MJCF Model (`so_arm101_model.xml`)** — The robot description file you can reuse in any MuJoCo project.

No GPU is required — everything runs on CPU.

---

## File Overview

| File | Purpose |
|------|---------|
| `setup_and_run.sh` | One-command installer: installs all Python dependencies and optionally launches the keyboard simulation |
| `01_mujoco_basics.ipynb` | Jupyter notebook covering MuJoCo basics through SO-ARM101 forward kinematics (5 code cells + 1 bonus) |
| `so_arm101_keyboard_sim.py` | Interactive keyboard-driven simulation with a live MuJoCo 3D viewer |
| `so_arm101_model.xml` | Standalone MJCF (XML) robot model for the SO-ARM101 |
| `README.md` | This file |

---

## Packages to Install

Run this once in your terminal:

```bash
pip install mujoco "gymnasium[mujoco]" matplotlib numpy ipywidgets jupyterlab requests
```

Or use the provided script which handles everything automatically:

```bash
bash setup_and_run.sh
```

**Minimum versions tested:**

| Package | Version |
|---------|---------|
| `mujoco` | ≥ 3.1 |
| `numpy` | ≥ 1.24 |
| `matplotlib` | ≥ 3.7 |
| `ipywidgets` | ≥ 8.0 |
| `jupyterlab` | ≥ 4.0 |

---

## How to Run

### Option A — Jupyter Notebook (Task 1.1)

```bash
jupyter lab 01_mujoco_basics.ipynb
```

Run cells top to bottom. The setup cell at the top auto-installs any missing packages.

### Option B — Interactive Keyboard Simulation

```bash
python so_arm101_keyboard_sim.py
```

A MuJoCo 3D viewer window will open. Use the keyboard to control the arm (see controls below).

### Option C — Quick Sanity Check

```bash
python -c "import mujoco; print(mujoco.__version__)"
python -m mujoco.viewer
```

The first command prints the installed version. The second opens MuJoCo's built-in viewer with a default scene.

---

## Keyboard Controls (Interactive Simulation)

| Key | Action |
|-----|--------|
| `Q` / `A` | Joint 1 — Base rotation (left / right) |
| `W` / `S` | Joint 2 — Shoulder pitch (up / down) |
| `E` / `D` | Joint 3 — Elbow (flex / extend) |
| `R` / `F` | Joint 4 — Wrist pitch (up / down) |
| `T` / `G` | Joint 5 — Wrist roll (clockwise / counter-clockwise) |
| `Y` / `H` | Gripper — Open / Close |
| `1` | Preset: Home (all joints at zero) |
| `2` | Preset: Elbow-up pose |
| `3` | Preset: Reach forward |
| `4` | Preset: Reach left |
| `5` | Preset: Reach right |
| `Space` | Toggle physics mode (PD position control ↔ free-fall) |
| `P` | Print current joint angles and TCP (X, Y, Z) position to terminal |
| `ESC` | Quit |

---

## What You Will See

### Notebook — Cell by Cell

**Cell 1 — Model Summary**
Prints a table of the model's degrees of freedom, body count, joint names, and joint limits. Example:

```
MODEL SUMMARY
══════════════════════════════════════════════
  nq   (generalized positions):  7
  nv   (generalized velocities): 7
  nbody (rigid bodies):          9
  njnt  (joints):                7
  nactuator (actuators):         7
══════════════════════════════════════════════
Joints:
  [0] joint1_rotation     range: [-3.14, 3.14] rad
  [1] joint2_pitch        range: [-1.57, 1.57] rad
  ...
```

**Cell 2 — Rendered Frame**
Displays a side-by-side matplotlib figure of the model rendered from two camera angles (CPU offscreen rendering, saved as `render_frame.png`).

**Cell 3 — Joint Positions Over Time**
Runs 1000 simulation steps with a random initial velocity and plots all joint positions as time-series curves (saved as `joint_positions.png`). Shows how the physics engine propagates motion.

**Cell 4 — SO-ARM101 Load & Render**
Loads the SO-ARM101 MJCF, prints the full model summary including all 7 actuators, and renders the arm in its rest pose (saved as `so_arm101_rest.png`).

**Cell 5 — Forward Kinematics & Workspace**
Evaluates the TCP (tool center point) position for several named poses and prints a table:

```
Config               X        Y        Z
────────────────────────────────────────────
Home (zero)      0.0000   0.0000   0.5050
Elbow up        -0.0000   0.0000   0.4520
Reach forward    0.2841   0.0000   0.3610
...
```

Then samples 2000 random joint configurations and plots the reachable workspace as a 3D scatter plot and an XZ projection (saved as `so_arm101_workspace.png`).

**Bonus Cell — Interactive Sliders**
If `ipywidgets` is installed, renders a live image that updates as you drag sliders for each joint. Also shows the TCP position above the rendered image. Falls back to a static 6-pose grid if widgets are unavailable (saved as `so_arm101_poses.png`).

### Keyboard Simulation — Terminal Output

When you press `P` in the viewer:

```
─── Current State ──────────────────────────────
  joint1_rotation              q=+0.3100  target=+0.3100
  joint2_pitch                 q=+0.5000  target=+0.5000
  joint3_elbow                 q=-0.8000  target=-0.8000
  joint4_wrist_pitch           q=+0.3000  target=+0.3000
  joint5_wrist_roll            q=+0.0000  target=+0.0000
  gripper_left                 q=+0.0000  target=+0.0000
  gripper_right                q=+0.0000  target=+0.0000

  TCP position: X=0.2341  Y=0.1023  Z=0.3871
────────────────────────────────────────────────
```

When you load a preset:

```
  → Preset: Elbow Up
```

When you toggle physics:

```
  → Mode: FREE-FALL physics
```

---

## SO-ARM101 Model Specification

| Property | Value |
|----------|-------|
| DOF | 6 (5 revolute + 1 parallel gripper) |
| Total height (extended) | ~0.56 m |
| Actuator type | Position-controlled (PD) |
| Timestep | 2 ms |
| End-effector sensor | Position (XYZ) + Quaternion |
| GPU required | No |

**Joint ranges:**

| Joint | Axis | Range |
|-------|------|-------|
| joint1_rotation | Z | ±180° |
| joint2_pitch | Y | ±90° |
| joint3_elbow | Y | ±150° |
| joint4_wrist_pitch | Y | ±90° |
| joint5_wrist_roll | Z | ±180° |
| gripper (each finger) | slide | 0–25 mm |

---

## References

- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [MuJoCo Menagerie — robot model collection](https://github.com/google-deepmind/mujoco_menagerie)
- [MuJoCo Playground](https://playground.mujoco.org/)
- [LeRobot / SO-ARM101](https://github.com/huggingface/lerobot)
- [TheRobotStudio SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100)
