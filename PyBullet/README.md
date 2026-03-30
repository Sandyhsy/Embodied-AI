# LeRobot SO-ARM100 PyBullet Simulation

A physics-based simulation environment for the **LeRobot SO-ARM100** robotic arm using PyBullet.

This project provides an interactive simulation where users can:

- control robot joints using a GUI slider interface
- visualize robot motion in real time
- record and replay movement trajectories
- save and load trajectories from JSON files

No physical robot is required.

---

## What This Project Does

This simulator creates a controllable robotic arm environment using PyBullet.

Users can directly manipulate joint angles through UI sliders and observe how the robot moves.

The system also supports trajectory recording and playback, making it useful for:

- motion testing
- demonstration
- control prototyping
- simple data collection

---

## Features

- PyBullet GUI simulation
- automatic detection of movable joints
- slider-based joint control
- record current pose
- save trajectory to JSON
- load trajectory from JSON
- replay trajectory smoothly
- reset robot to home pose
- print current joint values

---

## Requirements

- Python 3.8 or above
- pip

### Install dependencies

```bash
pip install pybullet