"""
SO-ARM101 MuJoCo Keyboard-Controlled Simulation
================================================
Run:  python so_arm101_keyboard_sim.py

Keyboard Controls:
──────────────────────────────────────────────────
  Joint Control (hold key for continuous motion):
    Q / A  →  Joint 1 (Shoulder Rotation)  +/-
    W / S  →  Joint 2 (Shoulder Pitch)     +/-
    E / D  →  Joint 3 (Elbow)              +/-
    R / F  →  Joint 4 (Wrist Pitch)        +/-
    T / G  →  Joint 5 (Wrist Roll)         +/-
    Y / H  →  Gripper                      open/close

  Presets:
    1  →  Home position (all zeros)
    2  →  Elbow-up pose
    3  →  Reach forward
    4  →  Reach left
    5  →  Reach right

  Simulation:
    SPACE  →  Toggle physics (free-fall vs. position hold)
    P      →  Print current joint angles & TCP position
    ESC    →  Quit
──────────────────────────────────────────────────
"""

import mujoco
import mujoco.viewer
import numpy as np
import threading
import time

# ─── SO-ARM101 MJCF ───────────────────────────────────────────────────────────
SO_ARM101_XML = '''
<mujoco model="so_arm101">
  <compiler angle="radian" coordinate="local"/>
  <option gravity="0 0 -9.81" timestep="0.002"/>

  <default>
    <joint limited="true" damping="0.5" armature="0.01"/>
    <geom contype="1" conaffinity="1" friction="1 0.005 0.0001"/>
  </default>

  <asset>
    <material name="metal"   rgba="0.65 0.65 0.70 1"/>
    <material name="joint"   rgba="0.25 0.45 0.85 1"/>
    <material name="gripper" rgba="0.90 0.60 0.20 1"/>
    <material name="floor"   rgba="0.85 0.85 0.85 1"/>
    <material name="target"  rgba="0.2  0.9  0.3  0.5"/>
  </asset>

  <worldbody>
    <light pos="0 0 3"   dir="0 0 -1"   diffuse="0.8 0.8 0.8"/>
    <light pos="1 1 1.5" dir="-1 -1 -1" diffuse="0.4 0.4 0.4" specular="0.1 0.1 0.1"/>
    <geom name="floor" type="plane" size="1 1 0.1" material="floor" pos="0 0 -0.01"/>

    <!-- Base -->
    <body name="base" pos="0 0 0">
      <geom name="base_col"  type="cylinder" size="0.07 0.045" material="metal" pos="0 0 0.045"/>
      <geom name="base_ring" type="cylinder" size="0.08 0.005" material="joint"  pos="0 0 0.088"/>
      <inertial pos="0 0 0.045" mass="0.6" diaginertia="0.002 0.002 0.002"/>

      <!-- Joint 1: Base Rotation -->
      <body name="link1" pos="0 0 0.095">
        <joint name="joint1_rotation"  type="hinge" axis="0 0 1" range="-3.14159 3.14159"/>
        <geom name="j1_body" type="cylinder" size="0.045 0.055" material="metal"  pos="0 0 0.055"/>
        <geom name="j1_ring" type="cylinder" size="0.048 0.004" material="joint"  pos="0 0 0.108"/>
        <inertial pos="0 0 0.055" mass="0.32" diaginertia="0.0006 0.0006 0.0003"/>

        <!-- Joint 2: Shoulder Pitch -->
        <body name="link2" pos="0 0 0.115">
          <joint name="joint2_pitch" type="hinge" axis="0 1 0" range="-1.5708 1.5708"/>
          <geom name="j2_cap" type="capsule" size="0.032" fromto="0 0 0 0 0 0.140" material="metal"/>
          <inertial pos="0 0 0.070" mass="0.26" diaginertia="0.0004 0.0004 0.0001"/>

          <!-- Joint 3: Elbow -->
          <body name="link3" pos="0 0 0.140">
            <joint name="joint3_elbow" type="hinge" axis="0 1 0" range="-2.618 2.618"/>
            <geom name="j3_cap" type="capsule" size="0.026" fromto="0 0 0 0 0 0.120" material="metal"/>
            <inertial pos="0 0 0.060" mass="0.21" diaginertia="0.0003 0.0003 0.0001"/>

            <!-- Joint 4: Wrist Pitch -->
            <body name="link4" pos="0 0 0.120">
              <joint name="joint4_wrist_pitch" type="hinge" axis="0 1 0" range="-1.5708 1.5708"/>
              <geom name="j4_cap" type="capsule" size="0.020" fromto="0 0 0 0 0 0.085" material="metal"/>
              <inertial pos="0 0 0.042" mass="0.15" diaginertia="0.0001 0.0001 0.00005"/>

              <!-- Joint 5: Wrist Roll -->
              <body name="link5" pos="0 0 0.085">
                <joint name="joint5_wrist_roll" type="hinge" axis="0 0 1" range="-3.14159 3.14159"/>
                <geom name="j5_body" type="cylinder" size="0.018 0.028" material="joint" pos="0 0 0.028"/>
                <inertial pos="0 0 0.028" mass="0.10" diaginertia="0.00005 0.00005 0.00003"/>

                <!-- Gripper -->
                <body name="gripper_base" pos="0 0 0.060">
                  <geom name="grip_body" type="box" size="0.030 0.020 0.018" material="gripper" pos="0 0 0.010"/>
                  <inertial pos="0 0 0.010" mass="0.09" diaginertia="0.00003 0.00003 0.00002"/>

                  <body name="finger_left" pos="-0.022 0 0.030">
                    <joint name="gripper_left" type="slide" axis="-1 0 0" range="0 0.025"/>
                    <geom type="box" size="0.008 0.011 0.022" material="gripper" pos="0 0 0.013"/>
                    <inertial pos="0 0 0.013" mass="0.020" diaginertia="0.000005 0.000005 0.000002"/>
                  </body>

                  <body name="finger_right" pos="0.022 0 0.030">
                    <joint name="gripper_right" type="slide" axis="1 0 0" range="0 0.025"/>
                    <geom type="box" size="0.008 0.011 0.022" material="gripper" pos="0 0 0.013"/>
                    <inertial pos="0 0 0.013" mass="0.020" diaginertia="0.000005 0.000005 0.000002"/>
                  </body>

                  <site name="end_effector" pos="0 0 0.080" size="0.012" rgba="1 0 0 0.8"/>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>

  <actuator>
    <position name="act_j1" joint="joint1_rotation"   kp="60" gear="1"/>
    <position name="act_j2" joint="joint2_pitch"       kp="60" gear="1"/>
    <position name="act_j3" joint="joint3_elbow"       kp="60" gear="1"/>
    <position name="act_j4" joint="joint4_wrist_pitch" kp="40" gear="1"/>
    <position name="act_j5" joint="joint5_wrist_roll"  kp="30" gear="1"/>
    <position name="act_gl" joint="gripper_left"        kp="25" gear="1"/>
    <position name="act_gr" joint="gripper_right"       kp="25" gear="1"/>
  </actuator>

  <sensor>
    <framepos  name="tcp_pos"  objtype="site" objname="end_effector"/>
    <framequat name="tcp_quat" objtype="site" objname="end_effector"/>
  </sensor>
</mujoco>
'''

# ─── Named Preset Poses ────────────────────────────────────────────────────────
PRESETS = {
    '1': {
        'name': 'Home (zero)',
        'q':    [0.0,  0.0,   0.0,  0.0,  0.0, 0.000, 0.000],
    },
    '2': {
        'name': 'Elbow Up',
        'q':    [0.0, -0.5,   1.0, -0.5,  0.0, 0.000, 0.000],
    },
    '3': {
        'name': 'Reach Forward',
        'q':    [0.0,  0.8,  -0.8,  0.5,  0.0, 0.000, 0.000],
    },
    '4': {
        'name': 'Reach Left',
        'q':    [1.57, 0.5,  -0.8,  0.3,  0.0, 0.000, 0.000],
    },
    '5': {
        'name': 'Reach Right',
        'q':    [-1.57, 0.5, -0.8,  0.3,  0.0, 0.000, 0.000],
    },
}

# ─── Key → (joint_index, direction) ──────────────────────────────────────────
KEY_MAP = {
    'q': (0, +1),  'a': (0, -1),   # Joint 1 Rotation
    'w': (1, +1),  's': (1, -1),   # Joint 2 Pitch
    'e': (2, +1),  'd': (2, -1),   # Joint 3 Elbow
    'r': (3, +1),  'f': (3, -1),   # Joint 4 Wrist Pitch
    't': (4, +1),  'g': (4, -1),   # Joint 5 Wrist Roll
    'y': (5, +1),  'h': (5, -1),   # Gripper (both fingers)
}

JOINT_STEP = 0.03   # radians per key-press
GRIPPER_STEP = 0.002  # meters per key-press

class ArmController:
    def __init__(self, model, data):
        self.model  = model
        self.data   = data
        self.target = np.zeros(model.nq)   # target joint positions
        self.physics_free = False            # True = free-fall physics
        self._lock  = threading.Lock()

        # site id for end-effector
        self.site_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, 'end_effector'
        )

    def apply_key(self, key: str):
        """Increment target joint angle based on key press."""
        key = key.lower()

        # Preset
        if key in PRESETS:
            p = PRESETS[key]
            q = p['q']
            with self._lock:
                for i in range(min(len(q), self.model.nq)):
                    lo, hi = self.model.joint(i).range
                    self.target[i] = float(np.clip(q[i], lo, hi))
            print(f'\n  → Preset: {p["name"]}')
            return

        # Toggle physics
        if key == ' ':
            self.physics_free = not self.physics_free
            mode = 'FREE-FALL physics' if self.physics_free else 'POSITION control'
            print(f'\n  → Mode: {mode}')
            return

        # Print state
        if key == 'p':
            self.print_state()
            return

        # Joint movement
        if key in KEY_MAP:
            j_idx, direction = KEY_MAP[key]
            with self._lock:
                if j_idx < 5:   # revolute
                    lo, hi = self.model.joint(j_idx).range
                    self.target[j_idx] = float(np.clip(
                        self.target[j_idx] + direction * JOINT_STEP, lo, hi
                    ))
                else:           # gripper slides (joints 5 & 6)
                    lo, hi = self.model.joint(5).range
                    delta  = direction * GRIPPER_STEP
                    self.target[5] = float(np.clip(self.target[5] + delta, lo, hi))
                    self.target[6] = float(np.clip(self.target[6] + delta, lo, hi))

    def print_state(self):
        mujoco.mj_forward(self.model, self.data)
        names = [self.model.joint(i).name for i in range(self.model.njnt)]
        print('\n─── Current State ──────────────────────────────')
        for i, n in enumerate(names):
            q_cur = float(self.data.qpos[i])
            q_tgt = float(self.target[i])
            print(f'  {n:<28} q={q_cur:+.4f}  target={q_tgt:+.4f}')
        if self.site_id >= 0:
            p = self.data.site_xpos[self.site_id]
            print(f'\n  TCP position: X={p[0]:.4f}  Y={p[1]:.4f}  Z={p[2]:.4f}')
        print('────────────────────────────────────────────────')

    def step(self):
        """Called every simulation step — set actuator targets."""
        with self._lock:
            if not self.physics_free:
                self.data.ctrl[:] = self.target[:self.model.na]


def main():
    print(__doc__)

    model = mujoco.MjModel.from_xml_string(SO_ARM101_XML)
    data  = mujoco.MjData(model)
    ctrl  = ArmController(model, data)

    print('Starting MuJoCo Viewer...')
    print('(Close the viewer window or press ESC to quit)\n')

    def key_callback(keycode):
        """MuJoCo viewer key callback (receives integer keycode)."""
        # Map common keycodes
        keymap = {
            81: 'q', 65: 'a',    # Q/A
            87: 'w', 83: 's',    # W/S
            69: 'e', 68: 'd',    # E/D
            82: 'r', 70: 'f',    # R/F
            84: 't', 71: 'g',    # T/G
            89: 'y', 72: 'h',    # Y/H
            49: '1', 50: '2', 51: '3', 52: '4', 53: '5',  # 1-5
            32: ' ',   # SPACE
            80: 'p',   # P
        }
        if keycode in keymap:
            ctrl.apply_key(keymap[keycode])

    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        viewer.cam.azimuth   = 135
        viewer.cam.elevation = -25
        viewer.cam.distance  = 1.2
        viewer.cam.lookat[:] = [0, 0, 0.25]

        # Print controls reminder
        print('═'*55)
        print('  KEYBOARD CONTROLS')
        print('═'*55)
        print('  Q/A  Joint1 Rotation   W/S  Joint2 Pitch')
        print('  E/D  Joint3 Elbow      R/F  Joint4 Wrist Pitch')
        print('  T/G  Joint5 Wrist Roll Y/H  Gripper open/close')
        print('  1-5  Load preset pose  SPC  Toggle physics')
        print('  P    Print state       ESC  Quit')
        print('═'*55 + '\n')

        while viewer.is_running():
            ctrl.step()
            mujoco.mj_step(model, data)
            viewer.sync()

    print('\nViewer closed. Bye!')


if __name__ == '__main__':
    main()
