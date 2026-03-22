import json
import os
import sys
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState

# ---------------------------------------------------------------------------
# Project-root discovery — works from source tree and colcon install tree.
# Mirrors the pattern used in the Jupyter notebook (fw_kinematics.ipynb).
# ---------------------------------------------------------------------------

def _find_project_root(start: Path) -> Path:
    """Walk up from start until finding the directory containing ros_ws/ and assignment/."""
    for candidate in [start, *start.parents]:
        if (candidate / 'ros_ws').is_dir() and (candidate / 'assignment').is_dir():
            return candidate
    raise RuntimeError(f'Cannot locate project root (edubot/) from {start}.')


_PROJECT_ROOT   = _find_project_root(Path(__file__).resolve())
_KINEMATICS_DIR = str(_PROJECT_ROOT / 'assignment')

# Add assignment/ to sys.path so robot_kinematics can be imported as a plain module.
# Same approach as inv_kinematics.py and fw_kinematics.ipynb.
if _KINEMATICS_DIR not in sys.path:
    sys.path.insert(0, _KINEMATICS_DIR)

from robot_kinematics import RobotKinematics          # noqa: E402
from assignment.read_ee_position import EEPositionReader  # noqa: E402
from assignment.joint_state_setter import JointStateSetter  # noqa: E402

DEFAULT_TRAJ_FILE = os.path.join(
    get_package_share_directory('assignment'), 'config', 'trajectories.json'
)

JOINT_NAMES = [
    'Shoulder_Rotation',
    'Shoulder_Pitch',
    'Elbow',
    'Wrist_Pitch',
    'Wrist_Roll',
    'Gripper',
]


class TrajectoryFollower(Node):
    """Follow a pre-defined Cartesian trajectory by iterating through waypoints, defined inside conf/.

    For each waypoint:
    - The IK is solved for all valid joint configurations.
    - The solution closest to the current joint state is selected (smooth motion).
    - The joint target is published to joint_states (→ robot_state_publisher → RViz).
    - The EE position is read from the TF tree to check if the waypoint is reached.
    """

    def __init__(self):
        super().__init__('trajectory_follower')

    
        traj_file  = DEFAULT_TRAJ_FILE
        publish_rate = 10

        # Import robot kinematics
        self.robot = RobotKinematics()
        self.get_logger().info('RobotKinematics correctly.')

        # ---- Trajectory state ----
        # All IK solutions stored per waypoint so the closest can be chosen at runtime.
        self._all_solutions     = []  # list[list[list[float]]]  — solutions per waypoint
        self._cartesian_targets = []  # list[np.ndarray(3,)]     — target EE positions
        self._loop              = True
        self._threshold         = 0.015
        self._waypoint_idx      = 0
        self._done              = False
        self._current_q         = None  # np.ndarray(5,) — updated from joint_states

        self.load_trajectory(traj_file)

        if not self._all_solutions:
            self.get_logger().fatal('No valid waypoints found — node cannot proceed.')
            raise RuntimeError('No valid waypoints.')



        # ---- ROS interfaces ----

        # Initiate setter and Reader Nodes
        self.reader = EEPositionReader(self)
        self.setter = JointStateSetter(self)

        # Warm up: spin until TF tree is available via the reader.
        print("Waiting for TF tree... ", end='', flush=True)
        self.reader.read()

        # Subscribe to joint_states to track current configuration for IK solution selection.
        self._sub = self.create_subscription(
            JointState, 'joint_states', self._on_joint_states, 10
        )
        self.create_timer(1.0 / publish_rate, self._timer_callback)

        self.get_logger().info(
            f'Trajectory follower ready — {len(self._all_solutions)} waypoints, '
            f'loop={self._loop}, threshold={self._threshold * 100:.1f} cm'
        )


    def load_trajectory(self, traj_file: str):
        self.get_logger().info(f'Loading trajectory: {traj_file}')

        with open(traj_file, 'r') as f:
            data = json.load(f)

        active_id = data['active_trajectory']
        if active_id not in data['trajectories']:
            raise KeyError(
                f'active_trajectory "{active_id}" not found. '
                f'Available: {list(data["trajectories"].keys())}'
            )

        traj = data['trajectories'][active_id]
        self.get_logger().info(f'Active trajectory: "{active_id}" — {traj["description"]}')

        self._loop      = traj.get('loop', True)                # Indicates whether trajectory should repeat or not
        self._threshold = traj.get('position_threshold', 0.015) # Indicates if position has been reached
        pitch, roll = 0.0, 0.0
        waypoints = traj['waypoints']
        
        self.get_logger().info(
            f'Pre-computing IK for {len(waypoints)} waypoints '
            f'(pitch={pitch:.1f}rad, roll={roll:.1f}rad)...'
        )

        for i, wp in enumerate(waypoints):
            # Use inverse Kinematics defined in robotKinematics to compute joint configurations 
            target_pose = [wp['x'], wp['y'], wp['z'], 0.5]
            solutions = self.robot.inverse_kinematics(target_pose)

            if not solutions:
                self.get_logger().warn(
                    f'  WP {i:02d} [{wp["x"]:.3f}, {wp["y"]:.3f}, {wp["z"]:.3f}] '
                    f'UNREACHABLE — skipped'
                )
                continue

            self._all_solutions.append(solutions)
            self._cartesian_targets.append(np.array(target_pose[:3]))

        self.get_logger().info(
            f'Pre-computation done: {len(self._all_solutions)}/{len(waypoints)} waypoints valid.'
        )


    def chose_closest_solution(self, waypoint_idx: int):
        """
        Return the IK solution closest in joint space to the current configuration.
        When the current joint state is unknown (startup) or only one solution exists,
        the first solution is returned.
        """
        solutions = self._all_solutions[waypoint_idx]

        if self._current_q is None or len(solutions) == 1:
            return np.array(solutions[0])

        # Euclidean distance in joint space between each solution and current configuration
        distances = [
            np.linalg.norm(np.array(sol) - self._current_q)
            for sol in solutions
        ]
        return np.array(solutions[int(np.argmin(distances))])

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def _on_joint_states(self, msg: JointState) -> None:
        """Track the current joint configuration (arm joints only, no gripper)."""
        name_to_pos = dict(zip(msg.name, msg.position))
        self._current_q = np.array([name_to_pos.get(n, 0.0) for n in JOINT_NAMES[:5]])

    def _timer_callback(self) -> None:
        if self._done or not self._all_solutions:
            return

        # ---- Pick and publish the target joint configuration ----
        target_q = self.chose_closest_solution(self._waypoint_idx)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = JOINT_NAMES
        msg.position = list(target_q) + [0.0]  # gripper fixed at 0
        self.setter._pub.publish(msg)

        # ---- Check if waypoint has been reached using TF-based EE reader ----
        pose = self.reader.lookup()
        if pose is None:
            return  # TF not yet available

        current_pos = np.array(pose[:3])
        target_pos  = self._cartesian_targets[self._waypoint_idx]
        error = float(np.linalg.norm(current_pos - target_pos))

        if error < self._threshold:
            self.get_logger().info(
                f'Reached WP {self._waypoint_idx:02d} '
                f'[{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}] '
                f'(err={error * 100:.2f} cm)'
            )
            next_idx = self._waypoint_idx + 1

            if next_idx >= len(self._all_solutions):
                if self._loop:
                    self._waypoint_idx = 0
                    self.get_logger().info('Loop: restarting trajectory from WP 00.')
                else:
                    self._done = True
                    self.get_logger().info('Trajectory complete. Holding last position.')
            else:
                self._waypoint_idx = next_idx


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
