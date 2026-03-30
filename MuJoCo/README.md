# SO-ARM100 MuJoCo Simulation

A CPU-only physics simulation of the **TRS SO-ARM100** robot arm using the official [MuJoCo Menagerie](https://github.com/chernyadev/mujoco_menagerie/tree/add-so-arm100/trs_so_arm100) model. Includes a guided Jupyter notebook and a live keyboard-controlled 3D viewer.

---

## What Does This Project Do?

This simulation uses the **real MJCF model** (with actual STL meshes) from MuJoCo Menagerie, not a simplified stand-in. It provides three things:

**1. Jupyter Notebook (`01_mujoco_basics.ipynb`)** — A step-by-step walkthrough of MuJoCo fundamentals using the Menagerie SO-ARM100. Covers model loading, CPU offscreen rendering, simulation stepping, named pose rendering, and 3D workspace visualisation via forward kinematics.

**2. Keyboard Viewer (`so_arm101_keyboard_sim.py`)** — Opens a live MuJoCo 3D window where you drive each joint with keyboard keys, switch between preset poses, toggle physics, and read TCP (tool-center-point) position in the terminal.

**3. Model Downloader (`download_so_arm100.py`)** — Fetches the full SO-ARM100 model (XML + STL assets) from the Menagerie GitHub branch directly to your machine. Must be run before the notebook or viewer.

No GPU is required — everything runs on CPU.

---

## File Overview

| File | Purpose |
|------|---------|
| `download_so_arm100.py` | Downloads the real SO-ARM100 model from GitHub into `models/trs_so_arm100/` |
| `01_mujoco_basics.ipynb` | Jupyter notebook — 5 code cells covering MuJoCo basics through workspace analysis |
| `so_arm101_keyboard_sim.py` | Interactive keyboard simulation with live MuJoCo 3D viewer |
| `setup_and_run.sh` | One-command installer for all Python dependencies |
| `README.md` | This file |

> `so_arm101_model.xml` (the old hand-written approximation) is no longer used — the Menagerie model replaces it.

---

## Packages to Install

```bash
pip install mujoco "gymnasium[mujoco]" matplotlib numpy ipywidgets jupyterlab requests
```

Or use the provided script:

```bash
bash setup_and_run.sh
```

**Minimum versions tested:**

| Package | Minimum Version |
|---------|----------------|
| `mujoco` | ≥ 3.1 |
| `numpy` | ≥ 1.24 |
| `matplotlib` | ≥ 3.7 |
| `ipywidgets` | ≥ 8.0 |
| `jupyterlab` | ≥ 4.0 |

---

## How to Run

### Step 0 — Download the model (required once)

```bash
python download_so_arm100.py
```

This fetches the Menagerie SO-ARM100 from:
`https://github.com/chernyadev/mujoco_menagerie/tree/add-so-arm100/trs_so_arm100`

and saves it to `models/trs_so_arm100/` (XML + STL assets).

---

### Option A — Jupyter Notebook

```bash
jupyter lab 01_mujoco_basics.ipynb
```

Run cells from top to bottom. Each cell is self-contained and prints its own output.

---

### Option B — Interactive Keyboard Simulation

```bash
python so_arm101_keyboard_sim.py
```

A MuJoCo 3D viewer window opens immediately. Use the keyboard controls below.

---

### Option C — View the Raw Model

```bash
python -m mujoco.viewer --mjcf models/trs_so_arm100/scene.xml
```

Opens MuJoCo's built-in viewer with the SO-ARM100 scene and default mouse/keyboard navigation.

---

## Keyboard Controls (Interactive Simulation)

### Joint Control (each press = one step)

| Key | Joint | Action |
|-----|------|--------|
| `Y` / `H` | Joint 1 — Base Rotation | + / − |
| `O` / `L` | Joint 2 — Shoulder Pitch | + / − |
| `,` / `.` | Joint 3 — Elbow | + / − |
| `Q` / `Z` | Joint 4 — Wrist Pitch | + / − |
| `E` / `F` | Joint 5 — Wrist Roll | + / − |
| `C` / `V` | Joint 6 — Gripper / Jaw | open / close |

---

### Preset Poses

| Key | Description |
|-----|------------|
| `4` | Home (all joints reset to zero) |
| `5` | Elbow-up pose |
| `6` | Reach forward |
| `7` | Reach left |
| `8` | Reach right |
| `9` | Wave motion |

---

### Simulation Controls

| Key | Action |
|-----|--------|
| `Space` | Toggle physics mode (PD control ↔ free-fall) |
| `ESC` | Quit viewer |

---

### Notes

- Each key press increments or decrements the joint target position by a fixed step.
- Revolute joints use radians; prismatic joints use meters.
- The controller updates actuator targets at every simulation step.
- Joint limits are automatically enforced using MuJoCo joint ranges.

---

## What You Will See

### `download_so_arm100.py`

```
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  SO-ARM100 MuJoCo Menagerie — Model Downloader
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
▶  Fetching file list from GitHub API...
   Found 4 items at top level.

  ✓  models/trs_so_arm100/scene.xml      (1,204 bytes)
  ✓  models/trs_so_arm100/so_arm100.xml  (8,931 bytes)
  ✓  models/trs_so_arm100/assets/base.stl        (25,600 bytes)
  ✓  models/trs_so_arm100/assets/link1.stl       (18,432 bytes)
  ...

▶  Testing model load in MuJoCo...
    Loaded scene.xml
      nq=6  nv=6  nbody=8
      Joints:
        [0] Rotation
        [1] Pitch
        ...
```

---

### Notebook — Cell by Cell

**Cell 1 — Model Summary**

Prints a formatted table of the model's DOF, body count, joint names and ranges, actuator names and gear ratios, and full body tree with parent links. Example excerpt:

```
╔════════════════════════════════════════════════════╗
║        TRS SO-ARM100 — Model Summary               ║
╠════════════════════════════════════════════════════╣
║  nq   (generalized positions)  : 6                 ║
║  nbody (rigid bodies)          : 8                 ║
║  na    (actuators)             : 6                 ║
║  nmesh  (meshes)               : 8                 ║
╠════════════════════════════════════════════════════╣
║  Joint name                     Range (rad/m)      ║
╠════════════════════════════════════════════════════╣
║  [0] Rotation           [-3.14, +3.14]             ║
║  [1] Pitch              [-1.57, +1.57]             ║
...
```

**Cell 2 — Four-View Render**

Saves `so_arm100_views.png` — a 4-panel matplotlib figure showing the SO-ARM100 in its rest pose from Front-Left, Front-Right, Side (Y), and Top-Down camera angles. Rendered entirely on CPU using MuJoCo's offscreen renderer.

**Cell 3 — Dynamics Plot**

Runs 1000 simulation steps with a small random velocity kick and saves `so_arm100_dynamics.png` — two stacked plots of joint positions (rad) and joint velocities (rad/s) over time, one curve per joint.

**Cell 4 — Named Pose Grid**

Saves `so_arm100_poses.png` — a 2×3 grid of rendered images showing six preset configurations: Home, Elbow Up, Reach Forward, Reach Left, Reach Right, and Wave.

**Cell 5 — Forward Kinematics & Workspace**

Prints a table of TCP (X, Y, Z) position and quaternion for each named pose, then samples 3000 random joint configurations and saves `so_arm100_workspace.png` — a three-panel figure with a 3D reachable workspace scatter plot, XZ side-view projection, and XY top-view projection.

**Bonus Cell — Interactive Sliders**

If `ipywidgets` is installed, displays a side-by-side layout of joint sliders and a live-rendered image that updates as you drag. The title above the image shows the current TCP position in metres. If `ipywidgets` is missing, prints an install instruction.

---

### Keyboard Simulation — Terminal Output

On startup:
```
Auto-detected joints:
  J1 (rotation)      → "Rotation"
  J2 (pitch)         → "Pitch"
  J3 (elbow)         → "Elbow"
  J4 (wrist_p)       → "Wrist_Pitch"
  J5 (wrist_r)       → "Wrist_Roll"
  J6 (gripper)       → "Jaw"

End-effector site: [3] "end_effector"
```

Press **P**:
```
─── Current State ─────────────────────────────────────────
  Rotation                       q=+0.3100 rad   target=+0.3100
  Pitch                          q=+0.5000 rad   target=+0.5000
  Elbow                          q=-0.8000 rad   target=-0.8000
  Wrist_Pitch                    q=+0.3000 rad   target=+0.3000
  Wrist_Roll                     q=+0.0000 rad   target=+0.0000
  Jaw                            q=+0.0000 rad   target=+0.0000

  TCP position:  X=0.2341  Y=0.1023  Z=0.3871 m
────────────────────────────────────────────────────────────
```

Press **Space**:
```
  → Physics mode: FREE-FALL (gravity, no control)
```

---

## Source Model

| Property | Detail |
|----------|--------|
| Repository | `chernyadev/mujoco_menagerie` |
| Branch | `add-so-arm100` |
| Folder | `trs_so_arm100/` |
| Full URL | https://github.com/chernyadev/mujoco_menagerie/tree/add-so-arm100/trs_so_arm100 |
| Manufacturer | The Robot Studio (TRS) |
| DOF | 6 (5 revolute + 1 jaw/gripper) |
| Geometry | Real STL meshes |
| Actuator type | Position-controlled |

---

## References

- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [MuJoCo Menagerie — robot model collection](https://github.com/google-deepmind/mujoco_menagerie)
- [TRS SO-ARM100 Menagerie branch](https://github.com/chernyadev/mujoco_menagerie/tree/add-so-arm100/trs_so_arm100)
- [MuJoCo Playground](https://playground.mujoco.org/)
- [LeRobot / SO-ARM series](https://github.com/huggingface/lerobot)
