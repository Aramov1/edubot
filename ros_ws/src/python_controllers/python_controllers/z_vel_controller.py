"""
XZ-plane Cartesian velocity controller using Jacobian pseudo-inverse.

The robot end-effector tracks a straight-line trajectory in the Z direction
at constant velocity while holding X and Y fixed.

Desired Cartesian velocity:
    x_dot = [vx=0,  vy=Ky*(Y_target-y),  vz=V_z,  vpitch=0,  vroll=0]

Joint velocities are computed as:
    q_dot = pinv(J(q)) @ x_dot

Usage:
    ros2 run python_controllers xz_velocity_controller \
        --ros-args -p velocity_z:=0.03 -p target_y:=0.20 -p z_min:=0.05 -p z_max:=0.25
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from python_controllers.robot_kinematics import RobotKinematics

JOINT_NAMES = [
    'Shoulder_Rotation',
    'Shoulder_Pitch',
    'Elbow',
    'Wrist_Pitch',
    'Wrist_Roll',
    'Gripper',
]

DEFAULT_KINEMATICS_PATH = '/home/andre/TU_Delft/edubot/assignment'


class ZVelController(Node):
    """
    ROS2 node that implements a Cartesian velocity controller for the robotic arm along the Z-axis, while maintaining a constant Y position. 
    The controller uses the Jacobian pseudo-inverse to compute joint velocities from the desired end-effector velocity.
    """
    def __init__(self):
        super().__init__('z_vel_controller')
        self.get_logger().info('Loading kinematics and initializing controller...')
        
        self.robot = RobotKinematics()
        self._direction  = 1.0    # +1 → moving up, -1 → moving down
        self._current_joint_angles  = None   # latest joint positions (5,)

        # ---- ROS interfaces ----
        self._pub = self.create_publisher(JointTrajectory, 'joint_cmds', 10)
        self._sub = self.create_subscription(JointState, 'joint_states', self._read_joint_states, qos_profile_sensor_data)
        self.create_timer(1.0 / 25, self._vel_control_loop)



        self._vel_z   =  0.02
        self._tgt_y   =  0.0
        self._tgt_x   =  0.0
        self._gain    =  2.0
        self._z_min   =  0.05
        self._z_max   =  0.25
        self._max_vel =  5.0

        self.get_logger().info(
            f'XZ velocity controller ready.\n'
            f'  velocity_z  = {self._vel_z:+.3f} m/s\n'
            f'  target_y    = {self._tgt_y:.3f} m\n'
            f'  target_x    = {self._tgt_x:.3f} m\n'
            f'  Z bounds    = [{self._z_min:.3f}, {self._z_max:.3f}] m\n'
            f'  max_joint_v = {self._max_vel:.2f} rad/s'
        )

        """
        # ---- Parameters ----
        self.declare_parameter('kinematics_path', DEFAULT_KINEMATICS_PATH)
        self.declare_parameter('velocity_z',      0.02)   # m/s — constant Z velocity
        self.declare_parameter('target_y',        0.20)   # m   — desired constant Y
        self.declare_parameter('y_gain',          2.0)    # proportional gain for Y correction
        self.declare_parameter('z_min',           0.05)   # m   — lower Z bound (reversal point)
        self.declare_parameter('z_max',           0.25)   # m   — upper Z bound (reversal point)
        self.declare_parameter('publish_rate',    25.0)   # Hz
        self.declare_parameter('max_joint_vel',   0.5)    # rad/s — per-joint velocity clip
        self.declare_parameter('cond_threshold',  200.0)  # warn if Jacobian is near-singular

        kin_path      = self.get_parameter('kinematics_path').value
        self._vel_z   = self.get_parameter('velocity_z').value
        self._tgt_y   = self.get_parameter('target_y').value
        self._gain    = self.get_parameter('y_gain').value
        self._z_min   = self.get_parameter('z_min').value
        self._z_max   = self.get_parameter('z_max').value
        rate          = self.get_parameter('publish_rate').value
        self._max_vel = self.get_parameter('max_joint_vel').value
        self._cond_th = self.get_parameter('cond_threshold').value
        """
    

    # Subscriber callback
    def _read_joint_states(self, msg: JointState) -> None:
        self._current_joint_angles = np.array(msg.position[:5])


    # Control loop + Velocity Commands Publisher
    def _vel_control_loop(self) -> None:
        if self._current_joint_angles is None:
            self.get_logger().warn('No joint state received yet, cannot compute control command.')
            return
        # Compute current EE pose using FK
        ee_pose = np.array(self.robot.forward_kinematics(*self._current_joint_angles)).flatten()
        x_cur, y_cur, z_cur = ee_pose[0], ee_pose[1], ee_pose[2]

        # Verify Z Limits bounds and reverse movement direction
        if z_cur >= self._z_max and self._direction > 0.0:
            self._direction = -1.0
            self.get_logger().info(f'Reached Z_max={self._z_max:.3f} m — reversing direction.')
        elif z_cur <= self._z_min and self._direction < 0.0:
            self._direction = 1.0
            self.get_logger().info(f'Reached Z_min={self._z_min:.3f} m — reversing direction.')

        # Compute desired EE velocity in Cartesian space
        vz = self._direction * self._vel_z
        vy = self._gain * (self._tgt_y - y_cur)  # proportional Y correction
        vx = self._gain * (self._tgt_x - x_cur)
        x_dot = np.array([vx, vy, vz, 0.0, 0.0])

        # Compute desired EE velocity in joint space
        J_inv, _ = self.robot.jacobian_inverse(self._current_joint_angles)
        q_dot = J_inv @ x_dot

        # Clamp joint velocity to the safety limit
        q_dot = np.clip(q_dot, -self._max_vel, self._max_vel)

        # Build and publish velocity command 
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names  = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.velocities = list(q_dot) + [0.0]   # append gripper velocity = 0
        msg.points = [point]
        self._pub.publish(msg)

        self.get_logger().info(
            f'EE: x={x_cur:+.3f}  y={y_cur:+.3f}  z={z_cur:+.3f} m | '
            f'vz={vz:+.4f} m/s | q_dot={np.round(q_dot, 3)}',
            throttle_duration_sec=0.5
        )


def main(args=None):
    rclpy.init(args=args)

    z_vel_controller_node = ZVelController()

    rclpy.spin(z_vel_controller_node)

    z_vel_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
