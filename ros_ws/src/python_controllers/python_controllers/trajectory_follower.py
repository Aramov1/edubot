import json
import os
import sys

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINT_NAMES = [
    'Shoulder_Rotation',
    'Shoulder_Pitch',
    'Elbow',
    'Wrist_Pitch',
    'Wrist_Roll',
    'Gripper',
]

DEFAULT_KINEMATICS_PATH = '/home/andre/TU_Delft/edubot/assignment'
DEFAULT_TRAJ_FILE = os.path.join(
    get_package_share_directory('python_controllers'), 'config', 'trajectories.json'
)


class TrajectoryFollower(Node):

    def __init__(self):
        super().__init__('trajectory_follower')

        # ---- Parameters ----
        self.declare_parameter('trajectory_file',  DEFAULT_TRAJ_FILE)
        self.declare_parameter('kinematics_path',  DEFAULT_KINEMATICS_PATH)
        self.declare_parameter('publish_rate',     10.0)
        self.declare_parameter('gripper',          0.0)

        traj_file   = self.get_parameter('trajectory_file').value
        kin_path    = self.get_parameter('kinematics_path').value
        publish_rate = self.get_parameter('publish_rate').value
        self._gripper = self.get_parameter('gripper').value

        # ---- Load kinematics module ----
        if kin_path not in sys.path:
            sys.path.insert(0, kin_path)
        try:
            from robot_kinematics import RobotKinematics  # noqa: PLC0415
        except ImportError as e:
            self.get_logger().fatal(
                f'Cannot import RobotKinematics from "{kin_path}": {e}\n'
                f'Set the kinematics_path parameter to the directory containing robot_kinematics.py'
            )
            raise

        self.get_logger().info('Initialising RobotKinematics (this may take a moment)...')
        self._kin = RobotKinematics()
        self.get_logger().info('RobotKinematics ready.')

        # ---- Load trajectory and pre-compute IK ----
        self._joint_targets     = []   # list of np.array(5,) — one per valid waypoint
        self._cartesian_targets = []   # list of np.array(3,) — matching positions
        self._loop              = True
        self._threshold         = 0.015
        self._waypoint_idx      = 0
        self._done              = False
        self._current_q         = None

        self._load_trajectory(traj_file)

        if not self._joint_targets:
            self.get_logger().fatal('No valid waypoints found — node cannot proceed.')
            raise RuntimeError('No valid waypoints.')

        # ---- ROS interfaces ----
        self._pub = self.create_publisher(JointTrajectory, 'joint_cmds', 10)
        self._sub = self.create_subscription(
            JointState, 'joint_states', self._on_joint_states, 10
        )
        self.create_timer(1.0 / publish_rate, self._timer_callback)

        self.get_logger().info(
            f'Trajectory follower running — {len(self._joint_targets)} waypoints, '
            f'loop={self._loop}, threshold={self._threshold} m'
        )

    # ------------------------------------------------------------------
    # Trajectory loading
    # ------------------------------------------------------------------

    def _load_trajectory(self, traj_file: str) -> None:
        self.get_logger().info(f'Loading trajectory file: {traj_file}')

        with open(traj_file, 'r') as f:
            data = json.load(f)

        active_id = data['active_trajectory']
        if active_id not in data['trajectories']:
            raise KeyError(
                f'active_trajectory "{active_id}" not found in trajectories: '
                f'{list(data["trajectories"].keys())}'
            )

        traj = data['trajectories'][active_id]
        self.get_logger().info(
            f'Active trajectory: "{active_id}" — {traj["description"]}'
        )

        self._loop      = traj.get('loop', True)
        self._threshold = traj.get('position_threshold', 0.015)
        pitch = traj.get('orientation', {}).get('pitch', 0.0)
        roll  = traj.get('orientation', {}).get('roll',  0.0)

        waypoints = traj['waypoints']
        self.get_logger().info(
            f'Pre-computing IK for {len(waypoints)} waypoints '
            f'(pitch={np.rad2deg(pitch):.1f}°, roll={np.rad2deg(roll):.1f}°)...'
        )

        for i, wp in enumerate(waypoints):
            target_pose = [wp['x'], wp['y'], wp['z'], pitch, roll]
            q = self._kin.inverse_kinematics(target_pose)

            if np.any(np.isnan(q)):
                self.get_logger().warn(
                    f'  WP {i:02d} [{wp["x"]:.3f}, {wp["y"]:.3f}, {wp["z"]:.3f}] '
                    f'is UNREACHABLE — skipped'
                )
                continue

            # Verify FK error
            fk = self._kin.forward_kinematics(*q)
            pos_err = np.linalg.norm(np.array(fk[:3]) - np.array(target_pose[:3]))
            self.get_logger().info(
                f'  WP {i:02d} [{wp["x"]:.3f}, {wp["y"]:.3f}, {wp["z"]:.3f}] '
                f'→ q={np.round(q, 3)}  FK err={pos_err*1000:.1f} mm'
            )

            self._joint_targets.append(np.array(q))
            self._cartesian_targets.append(np.array(target_pose[:3]))

        self.get_logger().info(
            f'Pre-computation done: {len(self._joint_targets)}/{len(waypoints)} waypoints valid.'
        )

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def _on_joint_states(self, msg: JointState) -> None:
        self._current_q = list(msg.position[:5])

    def _timer_callback(self) -> None:
        if self._done or not self._joint_targets:
            return

        # ---- Publish current target ----
        target_q = self._joint_targets[self._waypoint_idx]
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names  = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = list(target_q) + [self._gripper]
        msg.points = [pt]
        self._pub.publish(msg)

        # ---- Check if waypoint has been reached ----
        if self._current_q is None:
            return

        fk = self._kin.forward_kinematics(*self._current_q)
        current_pos = np.array(fk[:3])
        target_pos  = self._cartesian_targets[self._waypoint_idx]
        error = float(np.linalg.norm(current_pos - target_pos))

        if error < self._threshold:
            self.get_logger().info(
                f'Reached WP {self._waypoint_idx:02d} '
                f'[{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}] '
                f'(err={error*1000:.1f} mm)'
            )
            next_idx = self._waypoint_idx + 1

            if next_idx >= len(self._joint_targets):
                if self._loop:
                    self._waypoint_idx = 0
                    self.get_logger().info('Loop: restarting trajectory from WP 00.')
                else:
                    self._done = True
                    self.get_logger().info('Trajectory complete (loop=false). Holding last position.')
            else:
                self._waypoint_idx = next_idx


def main(args=None):
    rclpy.init(args=args)

    trajectory_follower = TrajectoryFollower()

    rclpy.spin(trajectory_follower)

    trajectory_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
