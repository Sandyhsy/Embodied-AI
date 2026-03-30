import os
import json
import time
import argparse
import traceback
from typing import List, Dict, Any

import pybullet as p
import pybullet_data


class SOARM100Simulator:
    def __init__(
        self,
        urdf_path: str,
        fixed_base: bool = True,
        time_step: float = 1.0 / 240.0,
        trajectory_file: str = "trajectory.json"
    ):
        self.urdf_path = urdf_path
        self.fixed_base = fixed_base
        self.time_step = time_step
        self.trajectory_file = trajectory_file

        self.client_id = None
        self.robot_id = None
        self.plane_id = None

        self.movable_joints: List[Dict[str, Any]] = []
        self.slider_ids: List[int] = []

        self.home_pose: List[float] = []
        self.recorded_poses: List[List[float]] = []

        self.button_ids: Dict[str, int] = {}
        self.last_button_values: Dict[str, float] = {}

        self.status_text_id = None
        self.info_text_id = None

        self.is_playing_trajectory = False

        
    def connect(self) -> None:
        try:
            self.client_id = p.connect(p.GUI)
        except Exception:
            print("GUI mode failed, falling back to DIRECT mode")
            self.client_id = p.connect(p.DIRECT)
        
        if self.client_id < 0:
            raise RuntimeError("Failed to connect to PyBullet.")

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(self.time_step)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
        p.resetDebugVisualizerCamera(
            cameraDistance=1.5,
            cameraYaw=50,
            cameraPitch=-35,
            cameraTargetPosition=[0.0, 0.0, 0.3]
        )

    def load_world(self) -> None:
        self.plane_id = p.loadURDF("plane.urdf")

        try:
            self.robot_id = p.loadURDF(
                self.urdf_path,
                basePosition=[0, 0, 0],
                baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
                useFixedBase=self.fixed_base,
                flags=p.URDF_USE_INERTIA_FROM_FILE
            )
        except Exception as e:
            raise RuntimeError(f"Failed to load URDF: {self.urdf_path}\n{e}")

        self._collect_movable_joints()

        if len(self.movable_joints) == 0:
            raise RuntimeError("No movable joints were found in the loaded robot.")

        self.home_pose = [0.0 for _ in self.movable_joints]
        self.reset_to_pose(self.home_pose)

    def _collect_movable_joints(self) -> None:
        self.movable_joints.clear()

        num_joints = p.getNumJoints(self.robot_id)
        for joint_index in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, joint_index)
            joint_type = joint_info[2]

            if joint_type in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
                joint_name = joint_info[1].decode("utf-8")
                lower_limit = joint_info[8]
                upper_limit = joint_info[9]

                # Use fallback limits when URDF limits are invalid or missing
                if lower_limit > upper_limit:
                    lower_limit = -3.14
                    upper_limit = 3.14

                if lower_limit < -1e10 or upper_limit > 1e10:
                    lower_limit = -3.14
                    upper_limit = 3.14

                self.movable_joints.append({
                    "joint_index": joint_index,
                    "joint_name": joint_name,
                    "lower_limit": lower_limit,
                    "upper_limit": upper_limit
                })

    def create_ui(self) -> None:
        self.slider_ids.clear()

        for joint in self.movable_joints:
            slider_id = p.addUserDebugParameter(
                paramName=joint["joint_name"],
                rangeMin=joint["lower_limit"],
                rangeMax=joint["upper_limit"],
                startValue=0.0
            )
            self.slider_ids.append(slider_id)

        self.button_ids["record_pose"] = p.addUserDebugParameter("Record Pose", 1, 0, 0)
        self.button_ids["save_trajectory"] = p.addUserDebugParameter("Save Trajectory", 1, 0, 0)
        self.button_ids["load_trajectory"] = p.addUserDebugParameter("Load Trajectory", 1, 0, 0)
        self.button_ids["play_trajectory"] = p.addUserDebugParameter("Play Trajectory", 1, 0, 0)
        self.button_ids["clear_trajectory"] = p.addUserDebugParameter("Clear Recorded Poses", 1, 0, 0)
        self.button_ids["reset_home"] = p.addUserDebugParameter("Reset Home", 1, 0, 0)
        self.button_ids["print_pose"] = p.addUserDebugParameter("Print Current Pose", 1, 0, 0)

        for key, button_id in self.button_ids.items():
            self.last_button_values[key] = self.safe_read_parameter(button_id, f"button_{key}")

        self._draw_info_text()
        self._update_status_text("Simulation started")

    def _draw_info_text(self) -> None:
        info_text = (
            "Controls:\n"
            "1. Move sliders to control joints\n"
            "2. Record Pose to store the current pose\n"
            "3. Save Trajectory to write poses to JSON\n"
            "4. Load Trajectory to read poses from JSON\n"
            "5. Play Trajectory to animate the stored poses\n"
            "6. Reset Home to return to the home pose\n"
            "7. Print Current Pose to show joint values in terminal"
        )

        if self.info_text_id is not None:
            try:
                p.removeUserDebugItem(self.info_text_id)
            except Exception:
                pass

        self.info_text_id = p.addUserDebugText(
            text=info_text,
            textPosition=[0.5, -0.45, 0.7],
            textColorRGB=[0, 0, 0],
            textSize=1.2
        )

    def _update_status_text(self, message: str) -> None:
        if self.status_text_id is not None:
            try:
                p.removeUserDebugItem(self.status_text_id)
            except Exception:
                pass

        self.status_text_id = p.addUserDebugText(
            text=f"Status: {message}",
            textPosition=[0.5, -0.45, 0.95],
            textColorRGB=[1, 0, 0],
            textSize=1.4
        )

    def safe_read_parameter(self, param_id: int, name: str = "parameter") -> float:
        if not p.isConnected(self.client_id):
            raise RuntimeError("PyBullet is not connected.")

        try:
            return p.readUserDebugParameter(param_id)
        except Exception as e:
            raise RuntimeError(f"Failed to read {name} (id={param_id}): {e}")

    def get_current_slider_pose(self) -> List[float]:
        pose = []
        for i, slider_id in enumerate(self.slider_ids):
            pose.append(self.safe_read_parameter(slider_id, f"slider_{i}"))
        return pose

    def get_current_joint_states(self) -> List[float]:
        pose = []
        for joint in self.movable_joints:
            state = p.getJointState(self.robot_id, joint["joint_index"])
            pose.append(state[0])
        return pose

    def set_joint_targets(self, target_pose: List[float], max_force: float = 60.0) -> None:
        for joint_cfg, target in zip(self.movable_joints, target_pose):
            p.setJointMotorControl2(
                bodyUniqueId=self.robot_id,
                jointIndex=joint_cfg["joint_index"],
                controlMode=p.POSITION_CONTROL,
                targetPosition=target,
                force=max_force
            )

    def reset_to_pose(self, pose: List[float]) -> None:
        for joint_cfg, value in zip(self.movable_joints, pose):
            p.resetJointState(self.robot_id, joint_cfg["joint_index"], value)
            p.setJointMotorControl2(
                bodyUniqueId=self.robot_id,
                jointIndex=joint_cfg["joint_index"],
                controlMode=p.POSITION_CONTROL,
                targetPosition=value,
                force=60.0
            )

    def record_pose(self) -> None:
        pose = self.get_current_slider_pose()
        self.recorded_poses.append(pose[:])
        self._update_status_text(f"Pose recorded. Total poses: {len(self.recorded_poses)}")

    def clear_recorded_poses(self) -> None:
        self.recorded_poses.clear()
        self._update_status_text("Recorded poses cleared")

    def save_trajectory(self) -> None:
        data = {
            "robot": "LeRobot SO-ARM101",
            "joint_names": [j["joint_name"] for j in self.movable_joints],
            "poses": self.recorded_poses
        }

        with open(self.trajectory_file, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)

        self._update_status_text(
            f"Trajectory saved to {self.trajectory_file} with {len(self.recorded_poses)} poses"
        )

    def load_trajectory(self) -> None:
        if not os.path.exists(self.trajectory_file):
            self._update_status_text(f"Trajectory file not found: {self.trajectory_file}")
            return

        with open(self.trajectory_file, "r", encoding="utf-8") as f:
            data = json.load(f)

        poses = data.get("poses", [])
        if not isinstance(poses, list):
            self._update_status_text("Invalid trajectory format")
            return

        # Validate pose length
        valid_poses = []
        expected_len = len(self.movable_joints)

        for pose in poses:
            if isinstance(pose, list) and len(pose) == expected_len:
                valid_poses.append(pose)

        self.recorded_poses = valid_poses
        self._update_status_text(
            f"Trajectory loaded from {self.trajectory_file}. Total poses: {len(self.recorded_poses)}"
        )

    def play_trajectory(self, steps_per_pose: int = 240) -> None:
        if len(self.recorded_poses) == 0:
            self._update_status_text("No recorded poses to play")
            return

        self.is_playing_trajectory = True
        self._update_status_text("Playing trajectory")

        current_pose = self.get_current_joint_states()

        try:
            for target_pose in self.recorded_poses:
                for step in range(steps_per_pose):
                    alpha = (step + 1) / steps_per_pose
                    interpolated_pose = [
                        current + alpha * (target - current)
                        for current, target in zip(current_pose, target_pose)
                    ]
                    self.set_joint_targets(interpolated_pose, max_force=60.0)
                    p.stepSimulation()
                    next_time = time.perf_counter() + self.time_step
                    while time.perf_counter() < next_time:
                        pass

                current_pose = target_pose[:]

            self._update_status_text("Trajectory playback finished")
        finally:
            self.is_playing_trajectory = False

    def print_current_pose(self) -> None:
        pose = self.get_current_joint_states()
        print("Current joint pose:")
        for joint_cfg, value in zip(self.movable_joints, pose):
            print(f"{joint_cfg['joint_name']}: {value:.4f}")

        print("Pose list format:")
        print([round(v, 4) for v in pose])

        self._update_status_text("Current pose printed to terminal")

    def _button_pressed(self, key: str) -> bool:
        current_value = self.safe_read_parameter(self.button_ids[key], f"button_{key}")
        last_value = self.last_button_values[key]

        pressed = current_value != last_value
        self.last_button_values[key] = current_value
        return pressed

    def update_manual_control(self) -> None:
        if not p.isConnected(self.client_id):
            return

        if self.is_playing_trajectory:
            return

        target_pose = self.get_current_slider_pose()
        self.set_joint_targets(target_pose, max_force=60.0)

    def run(self) -> None:
        self.connect()
        self.load_world()
        self.create_ui()

        try:
            while p.isConnected(self.client_id):
                self.update_manual_control()

                if self._button_pressed("record_pose"):
                    self.record_pose()

                if self._button_pressed("save_trajectory"):
                    self.save_trajectory()

                if self._button_pressed("load_trajectory"):
                    self.load_trajectory()

                if self._button_pressed("play_trajectory"):
                    self.play_trajectory()

                if self._button_pressed("clear_trajectory"):
                    self.clear_recorded_poses()

                if self._button_pressed("reset_home"):
                    self.reset_to_pose(self.home_pose)
                    self._update_status_text("Robot reset to home pose")

                if self._button_pressed("print_pose"):
                    self.print_current_pose()

                p.stepSimulation()
                next_time = time.perf_counter() + self.time_step
                while time.perf_counter() < next_time:
                    pass

        except KeyboardInterrupt:
            print("Simulation stopped by user.")
        except Exception as e:
            print(f"Runtime error: {e}")
            traceback.print_exc()
        finally:
            if p.isConnected(self.client_id):
                p.disconnect()


def parse_args():
    parser = argparse.ArgumentParser(description="PyBullet UI simulation for LeRobot SO-ARM101")
    parser.add_argument(
        "--urdf",
        type=str,
        default="franka_panda/panda.urdf",
        help="Path to the robot URDF file"
    )
    parser.add_argument(
        "--trajectory",
        type=str,
        default="trajectory.json",
        help="Path to save or load trajectory JSON"
    )
    parser.add_argument(
        "--free-base",
        action="store_true",
        help="Load the robot without a fixed base"
    )
    return parser.parse_args()


def main():
    args = parse_args()

    simulator = SOARM100Simulator(
        urdf_path=args.urdf,
        fixed_base=not args.free_base,
        trajectory_file=args.trajectory
    )
    simulator.run()


if __name__ == "__main__":
    main()