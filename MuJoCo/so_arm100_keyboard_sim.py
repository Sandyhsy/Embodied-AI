"""
SO-ARM100 MuJoCo Keyboard-Controlled Simulation
================================================
Uses the official TRS SO-ARM100 model from MuJoCo Menagerie.

Run FIRST:
    python download_so_arm100.py

Then:
    python so_arm101_keyboard_sim.py

Keyboard Controls:
──────────────────────────────────────────────────────────────────
  Joint Control  (each press = one step; hold for continuous):
    Q / A  →  Joint 1  (Base Rotation)       +/-
    W / S  →  Joint 2  (Shoulder Pitch)      +/-
    E / D  →  Joint 3  (Elbow)               +/-
    R / F  →  Joint 4  (Wrist Pitch)         +/-
    T / G  →  Joint 5  (Wrist Roll)          +/-
    Y / H  →  Joint 6  (Gripper / Jaw)       open / close

  Preset Poses:
    4  →  Home (all zeros)
    5  →  Elbow-up
    6  →  Reach forward
    7  →  Reach left
    8  →  Reach right
    9  →  Wave

  Simulation:
    SPACE  →  Toggle physics mode (position-hold ↔ free-fall)
    P      →  Print current joint angles + TCP position
    ESC    →  Quit viewer
──────────────────────────────────────────────────────────────────
"""

import os
import sys
import mujoco
import mujoco.viewer
import numpy as np
import threading

# ─── Model path ───────────────────────────────────────────────────────────────
MODEL_DIR = os.path.join("models", "trs_so_arm100")
SCENE_XML = os.path.join(MODEL_DIR, "scene.xml")
ROBOT_XML = os.path.join(MODEL_DIR, "so_arm100.xml")


def load_model():
    """Load the Menagerie SO-ARM100 model, or exit with a helpful message."""
    for candidate in [SCENE_XML, ROBOT_XML]:
        if os.path.exists(candidate) and os.path.getsize(candidate) > 0:
            print(f"Loading model: {candidate}")
            return mujoco.MjModel.from_xml_path(candidate), candidate

    print()
    print("━"*60)
    print("  ✗  SO-ARM100 model not found.")
    print()
    print("  Run the downloader first:")
    print("      python download_so_arm100.py")
    print("━"*60)
    sys.exit(1)


# ─── Joint-name auto-detection ────────────────────────────────────────────────
def find_joint(model, keywords):
    """Return name of first joint whose name contains any keyword."""
    for kw in keywords:
        for i in range(model.njnt):
            if kw.lower() in model.joint(i).name.lower():
                return model.joint(i).name
    return None


def find_site(model, keywords):
    """Return site_id of first site matching any keyword; -1 if not found."""
    for kw in keywords:
        for i in range(model.nsite):
            if kw.lower() in model.site(i).name.lower():
                return i
    return model.nsite - 1 if model.nsite > 0 else -1


# ─── Preset poses ────────────────────────────────────────────────────────────
def build_presets(model, j1, j2, j3, j4, j5, j6):
    """
    Build a dict of preset name → {joint_name: angle}.
    Uses auto-detected joint names; None joints are skipped gracefully.
    """
    def p(**kw):
        return {k: v for k, v in kw.items() if k is not None}

    return {
        "4": ("Home",          p(**{j1: 0.0,  j2: 0.0,  j3: 0.0,  j4: 0.0,  j5: 0.0})),
        "5": ("Elbow Up",      p(**{j1: 0.0,  j2:-0.5,  j3: 1.0,  j4:-0.5,  j5: 0.0})),
        "6": ("Reach Forward", p(**{j1: 0.0,  j2: 0.8,  j3:-0.8,  j4: 0.5,  j5: 0.0})),
        "7": ("Reach Left",    p(**{j1: 1.57, j2: 0.5,  j3:-0.8,  j4: 0.3,  j5: 0.0})),
        "8": ("Reach Right",   p(**{j1:-1.57, j2: 0.5,  j3:-0.8,  j4: 0.3,  j5: 0.0})),
        "9": ("Wave",          p(**{j1: 0.5,  j2:-0.3,  j3: 0.8,  j4:-0.3,  j5: 1.0})),
    }


# ─── Controller ──────────────────────────────────────────────────────────────
class ArmController:

    JOINT_STEP   = 0.03   # radians / step
    SLIDE_STEP   = 0.002  # meters  / step

    def __init__(self, model, data):
        self.model  = model
        self.data   = data
        self._lock  = threading.Lock()

        # Target joint positions (position-control target)
        self.target = np.zeros(model.nq)

        # Physics mode: False = PD position control, True = free-fall
        self.physics_free = False

        # Auto-detect joints
        self.j1 = find_joint(model, ['rotation', 'base_rot', 'joint1', 'j1'])
        self.j2 = find_joint(model, ['pitch', 'shoulder', 'joint2', 'j2'])
        self.j3 = find_joint(model, ['elbow', 'joint3', 'j3'])
        self.j4 = find_joint(model, ['wrist_pitch', 'wrist', 'joint4', 'j4'])
        self.j5 = find_joint(model, ['wrist_roll', 'roll', 'joint5', 'j5'])
        self.j6 = find_joint(model, ['jaw', 'gripper', 'finger', 'joint6', 'j6'])

        print("\nAuto-detected joints:")
        for label, jname in [("J1 (rotation)", self.j1), ("J2 (pitch)",    self.j2),
                              ("J3 (elbow)",    self.j3), ("J4 (wrist_p)", self.j4),
                              ("J5 (wrist_r)",  self.j5), ("J6 (gripper)", self.j6)]:
            status = f'"{jname}"' if jname else "NOT FOUND"
            print(f"  {label:<20} → {status}")

        # Build key→(joint_name, direction) map
        self.key_joint_map = {
            'y': (self.j1, +1), 'h': (self.j1, -1),
            'o': (self.j2, +1), 'l': (self.j2, -1),
            ',': (self.j3, +1), '.': (self.j3, -1),
            'q': (self.j4, +1), 'z': (self.j4, -1),
            'e': (self.j5, +1), 'c': (self.j5, -1),
            'f': (self.j6, +1), 'v': (self.j6, -1),
        }

        # Presets
        self.presets = build_presets(
            model, self.j1, self.j2, self.j3, self.j4, self.j5, self.j6
        )

        # End-effector site
        self.site_id = find_site(
            model, ['end_effector', 'tcp', 'tool', 'eef', 'tip', 'gripper', 'attachment']
        )
        if self.site_id >= 0:
            sname = model.site(self.site_id).name
            print(f"\nEnd-effector site: [{self.site_id}] \"{sname}\"")

    # ── Internal helpers ──────────────────────────────────────────────────────
    def _joint_id(self, jname):
        if jname is None:
            return -1
        return mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jname)

    def _qadr(self, jname):
        jid = self._joint_id(jname)
        return self.model.jnt_qposadr[jid] if jid >= 0 else -1

    def _clamp_target(self, jname, val):
        jid = self._joint_id(jname)
        if jid < 0:
            return val
        lo, hi = self.model.joint(jid).range
        return float(np.clip(val, lo, hi))

    def _is_slide(self, jname):
        jid = self._joint_id(jname)
        return jid >= 0 and self.model.joint(jid).type == 2

    # ── Public API ────────────────────────────────────────────────────────────
    def apply_key(self, key: str):
        key = key.lower()

        # Preset poses
        if key in self.presets:
            pname, angles = self.presets[key]
            with self._lock:
                for jname, val in angles.items():
                    qadr = self._qadr(jname)
                    if qadr >= 0:
                        self.target[qadr] = self._clamp_target(jname, val)
            print(f"\n  → Preset: {pname}")
            return

        # Toggle physics
        if key == ' ':
            self.physics_free = not self.physics_free
            mode = "FREE-FALL (gravity, no control)" if self.physics_free else "POSITION CONTROL (PD)"
            print(f"\n  → Physics mode: {mode}")
            return

        # Print state
        if key == 'p':
            self.print_state()
            return

        # Joint increment
        if key in self.key_joint_map:
            jname, direction = self.key_joint_map[key]
            if jname is None:
                return
            step = self.SLIDE_STEP if self._is_slide(jname) else self.JOINT_STEP
            qadr = self._qadr(jname)
            if qadr >= 0:
                with self._lock:
                    self.target[qadr] = self._clamp_target(
                        jname, self.target[qadr] + direction * step
                    )

    def step(self):
        """Called every simulation tick — write actuator targets."""
        if not self.physics_free:
            with self._lock:
                # Map target qpos → actuator ctrl
                for i in range(self.model.nu):
                    a    = self.model.actuator(i)
                    jid  = a.trnid[0]   # joint this actuator drives
                    qadr = self.model.jnt_qposadr[jid] if jid >= 0 else -1
                    if qadr >= 0:
                        self.data.ctrl[i] = self.target[qadr]

    def print_state(self):
        mujoco.mj_forward(self.model, self.data)
        print("\n─── Current State " + "─"*43)
        for i in range(self.model.njnt):
            j    = self.model.joint(i)
            if j.type not in [2, 3]:
                continue
            qadr = self.model.jnt_qposadr[i]
            q_c  = float(self.data.qpos[qadr])
            q_t  = float(self.target[qadr])
            unit = "m" if j.type == 2 else "rad"
            print(f"  {j.name:<30} q={q_c:+.4f} {unit}   target={q_t:+.4f}")
        if self.site_id >= 0:
            p = self.data.site_xpos[self.site_id]
            print(f"\n  TCP position:  X={p[0]:.4f}  Y={p[1]:.4f}  Z={p[2]:.4f} m")
        print("─"*60)


# ─── Main ────────────────────────────────────────────────────────────────────
def main():
    print(__doc__)

    model, xml_path = load_model()
    data            = mujoco.MjData(model)
    ctrl            = ArmController(model, data)

    # Key-code → character mapping (MuJoCo uses GLFW key codes)
    KEYMAP = {
        89: 'y', 72: 'h',   # Y H
        79: 'o', 76: 'l',   # O L
        44: ',', 46: '.',   # , .
        81: 'q', 90: 'z',   # Q Z
        69: 'e', 70: 'c',   # E C
        67: 'f', 86: 'v',   # F V

        52: '4', 53: '5', 54: '6',
        55: '7', 56: '8', 57: '9',

        32: ' ',
    }

    def key_callback(keycode):
        if keycode in KEYMAP:
            ctrl.apply_key(KEYMAP[keycode])

    print("\n" + "═"*60)
    print("  Starting MuJoCo Viewer...")
    print("  (Close window or press ESC to quit)")
    print("═"*60 + "\n")

    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        # Initial camera
        viewer.cam.azimuth   = 135
        viewer.cam.elevation = -20
        viewer.cam.distance  = 1.5
        viewer.cam.lookat[:] = [0.0, 0.0, 0.22]

        print("\n" + "="*65)
        print("Keyboard Controls (Interactive Simulation)")
        print("="*65)

        print("\nJoint Control:")
        print("+-----+------------------------------+-----------+")
        print("| Key | Joint                        | Action    |")
        print("+-----+------------------------------+-----------+")
        print("| Y/H | Joint 1 — Base Rotation     | + / -     |")
        print("| O/L | Joint 2 — Shoulder Pitch    | + / -     |")
        print("| ,/. | Joint 3 — Elbow             | + / -     |")
        print("| Q/Z | Joint 4 — Wrist Pitch       | + / -     |")
        print("| E/F | Joint 5 — Wrist Roll        | + / -     |")
        print("| C/V | Joint 6 — Gripper / Jaw     | open/close|")
        print("+-----+------------------------------+-----------+")

        print("\nPreset Poses:")
        print("+-----+-----------------------------+")
        print("| Key | Description                 |")
        print("+-----+-----------------------------+")
        print("| 4   | Home (all zeros)           |")
        print("| 5   | Elbow-up                   |")
        print("| 6   | Reach forward              |")
        print("| 7   | Reach left                 |")
        print("| 8   | Reach right                |")
        print("| 9   | Wave                       |")
        print("+-----+-----------------------------+")

        print("\nSimulation Controls:")
        print("+--------+------------------------------------------+")
        print("| Key    | Action                                   |")
        print("+--------+------------------------------------------+")
        print("| Space  | Toggle physics (PD ↔ free-fall)          |")
        print("| ESC    | Quit viewer                              |")
        print("+--------+------------------------------------------+")

        while viewer.is_running():
            ctrl.step()
            mujoco.mj_step(model, data)
            viewer.sync()

    print("\nViewer closed.")


if __name__ == "__main__":
    main()
