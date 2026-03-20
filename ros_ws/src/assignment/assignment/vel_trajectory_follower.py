"""
Cartesian velocity trajectory follower using Jacobian pseudo-inverse.

Loads a set of Cartesian waypoints from a JSON config file and follows them
using velocity control: at each timestep the positional error to the current
waypoint is converted to a Cartesian velocity via proportional control, then
mapped to joint velocities through the Jacobian pseudo-inverse (DLS for
singularity robustness).

Waypoint transitions occur once the EE is within `position_threshold` metres
of the target.  When `loop` is true the trajectory restarts after the last
waypoint.

Usage:
    ros2 run assignment vel_trajectory_follower
"""

import json
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# ---------------------------------------------------------------------------
# Project-root discovery — mirrors trajectory_follower.py.
# ---------------------------------------------------------------------------

def _find_project_root(start: Path) -> Path:
    """Walk up from start until finding the directory containing ros_ws/ and assignment/."""
    for candidate in [start, *start.parents]:
        if (candidate / 'ros_ws').is_dir() and (candidate / 'assignment').is_dir():
            return candidate
    raise RuntimeError(f'Cannot locate project root (edubot/) from {start}.')


_PROJECT_ROOT = _find_project_root(Path(__file__).resolve())
_KINEMATICS_DIR = str(_PROJECT_ROOT / 'assignment')

if _KINEMATICS_DIR not in sys.path:
    sys.path.insert(0, _KINEMATICS_DIR)

from robot_kinematics import RobotKinematics  # noqa: E402

DEFAULT_TRAJ_FILE = os.path.join(
    get_package_share_directory('assignment'), 'config', 'vel_trajectories.json'
)

JOINT_NAMES = [
    'Shoulder_Rotation',
    'Shoulder_Pitch',
    'Elbow',
    'Wrist_Pitch',
    'Wrist_Roll',
    'Gripper',
]


class VelTrajectoryFollower(Node):
    """
    ROS 2 node that follows a Cartesian waypoint trajectory via velocity control.

    For each waypoint the positional error is converted to a desired Cartesian
    velocity (proportional control), which is then mapped to joint velocities
    using the Jacobian pseudo-inverse with Damped Least Squares (DLS) to keep
    the solution stable near singularities.
    """

    def __init__(self):
        super().__init__('vel_trajectory_follower')

        # Ensure the simulator is running in velocity mode.
        # lerobot_sim must be active to publish joint_states and accept velocity commands.
        self._sim_proc = None
        self._ensure_velocity_sim()

        # Load kinematics
        self.robot = RobotKinematics()
        self.get_logger().info('RobotKinematics loaded.')

        # Trajectory state
        self._waypoints = []      # list of np.ndarray(3,) — [x, y, z] targets
        self._waypoint_idx = 0
        self._loop = True
        self._threshold = 0.015   # m — distance to consider a waypoint reached
        self._gain = 3.0          # proportional gain: v_ee = gain * position_error
        self._max_joint_vel = 1.0 # rad/s — per-joint velocity safety clamp
        self._done = False

        self._current_q = None    # np.ndarray(5,) — updated from joint_states

        self._load_trajectory(DEFAULT_TRAJ_FILE)

        if not self._waypoints:
            self.get_logger().fatal('No waypoints loaded — node cannot proceed.')
            raise RuntimeError('No waypoints.')

        # ROS interfaces
        self._pub = self.create_publisher(JointTrajectory, 'joint_cmds', 10)
        self._sub = self.create_subscription(
            JointState, 'joint_states', self._on_joint_states, qos_profile_sensor_data
        )
        self.create_timer(1.0 / 25.0, self._control_loop)   # 25 Hz control loop

        self.get_logger().info(
            f'Velocity trajectory follower ready — {len(self._waypoints)} waypoints, '
            f'loop={self._loop}, threshold={self._threshold * 100:.1f} cm, '
            f'gain={self._gain:.1f}, max_joint_vel={self._max_joint_vel:.2f} rad/s'
        )

    # ------------------------------------------------------------------
    # Simulator management
    # ------------------------------------------------------------------

    def _ensure_velocity_sim(self) -> None:
        """Launch lerobot_sim in velocity mode if it is not already running."""
        try:
            result = subprocess.run(
                ['ros2', 'node', 'list'], capture_output=True, text=True, timeout=5
            )
            running_nodes = result.stdout
        except (subprocess.TimeoutExpired, FileNotFoundError):
            running_nodes = ''

        if 'lerobot_sim' in running_nodes:
            self.get_logger().info('[OK] lerobot_sim already running.')
            return

        self.get_logger().info(
            '[INFO] lerobot_sim not found — launching in velocity mode (no RViz)...'
        )
        self._sim_proc = subprocess.Popen(
            ['ros2', 'launch', 'lerobot', 'sim_velocity.launch.py', 'use_rviz:=false'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )

        # Wait up to 10 s for lerobot_sim to appear
        for _ in range(20):
            time.sleep(0.5)
            try:
                r = subprocess.run(
                    ['ros2', 'node', 'list'], capture_output=True, text=True, timeout=5
                )
                if 'lerobot_sim' in r.stdout:
                    self.get_logger().info('[OK] lerobot_sim is running in velocity mode.')
                    return
            except (subprocess.TimeoutExpired, FileNotFoundError):
                pass

        self.get_logger().error(
            '[ERROR] lerobot_sim did not start within 10 s. '
            'Run manually: ros2 launch lerobot sim_velocity.launch.py'
        )

    # ------------------------------------------------------------------
    # Trajectory loading
    # ------------------------------------------------------------------

    def _load_trajectory(self, traj_file: str) -> None:
        """Parse the JSON config and populate self._waypoints."""
        self.get_logger().info(f'Loading trajectory from: {traj_file}')

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

        # Optional per-trajectory overrides
        self._loop = traj.get('loop', True)
        self._threshold = traj.get('position_threshold', 0.015)
        self._gain = traj.get('gain', 3.0)
        self._max_joint_vel = traj.get('max_joint_vel', 1.0)

        for wp in traj['waypoints']:
            self._waypoints.append(np.array([wp['x'], wp['y'], wp['z']], dtype=float))

        self.get_logger().info(f'Loaded {len(self._waypoints)} waypoints.')

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def _on_joint_states(self, msg: JointState) -> None:
        """Track the current joint configuration (arm joints only, no gripper).

        Joint angles are clamped to the kinematics bounds so that FK and the
        Jacobian never receive an out-of-range value — the simulator can report
        positions slightly outside the modelled limits during fast motion.
        """
        name_to_pos = dict(zip(msg.name, msg.position))
        q_raw = np.array([name_to_pos.get(n, 0.0) for n in JOINT_NAMES[:5]])

        # Clamp each joint to its allowed range (order matches robot_kinematics.joint_bounds)
        bounds = list(self.robot.joint_bounds.values())
        self._current_q = np.array([
            np.clip(q_raw[i], bounds[i][0], bounds[i][1]) for i in range(5)
        ])

    def _control_loop(self) -> None:
        """Main velocity control step executed at 25 Hz."""
        if self._done:
            return

        if self._current_q is None:
            self.get_logger().warn('Waiting for joint states...', throttle_duration_sec=1.0)
            return

        # Current EE position from forward kinematics
        ee_pose = np.array(self.robot.forward_kinematics(*self._current_q)).flatten()
        pos_cur = ee_pose[:3]

        target_pos = self._waypoints[self._waypoint_idx]
        error_vec = target_pos - pos_cur
        error_norm = float(np.linalg.norm(error_vec))

        # Advance to next waypoint once within the position threshold
        if error_norm < self._threshold:
            self.get_logger().info(
                f'Reached WP {self._waypoint_idx:02d} '
                f'[{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}] '
                f'(err={error_norm * 100:.2f} cm)'
            )
            next_idx = self._waypoint_idx + 1
            if next_idx >= len(self._waypoints):
                if self._loop:
                    self._waypoint_idx = 0
                    self.get_logger().info('Loop: restarting from WP 00.')
                else:
                    self._done = True
                    self.get_logger().info('Trajectory complete. Stopping.')
                    self._publish_velocity(np.zeros(5))
                    return
            else:
                self._waypoint_idx = next_idx

        # Desired Cartesian velocity — proportional to positional error.
        # J maps 5 joint velocities → 6 Cartesian DOF [vx, vy, vz, vroll, vpitch, vyaw].
        # Orientation components are left at zero (hold current orientation).
        x_dot = np.zeros(6)
        x_dot[:3] = self._gain * error_vec

        # Map Cartesian velocity to joint velocities via Jacobian pseudo-inverse (DLS)
        J_inv, is_singular = self.robot.jacobian_inverse(self._current_q)
        if is_singular:
            self.get_logger().warn(
                'Near singularity detected — DLS damping active.', throttle_duration_sec=0.5
            )
        q_dot = J_inv @ x_dot

        # Clamp joint velocities to the safety limit
        q_dot = np.clip(q_dot, -self._max_joint_vel, self._max_joint_vel)

        self._publish_velocity(q_dot)

        self.get_logger().info(
            f'WP {self._waypoint_idx:02d} | err={error_norm * 100:.2f} cm | '
            f'q_dot={np.round(q_dot, 3)}',
            throttle_duration_sec=0.2
        )

    # ------------------------------------------------------------------
    # Publishing helpers
    # ------------------------------------------------------------------

    def _publish_velocity(self, q_dot: np.ndarray) -> None:
        """Publish a joint velocity command (5 arm joints + gripper fixed at 0)."""
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.velocities = list(q_dot) + [0.0]   # gripper velocity = 0
        msg.points = [point]
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VelTrajectoryFollower()
    rclpy.spin(node)
    node.destroy_node()
    # Terminate the simulator if this node launched it
    if node._sim_proc is not None:
        node._sim_proc.terminate()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
