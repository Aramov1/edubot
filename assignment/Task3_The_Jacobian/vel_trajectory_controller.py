import sys
import os
_ASSIGNMENT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_PROJECT_ROOT   = os.path.dirname(_ASSIGNMENT_DIR)
sys.path.insert(0, _ASSIGNMENT_DIR)
sys.path.insert(0, os.path.join(_PROJECT_ROOT, 'ros_ws', 'src', 'assignment'))

import threading
import rclpy
from rclpy.node import Node
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data
from robot_kinematics import RobotKinematics

class VelTrajectoryController(Node):
    def __init__(self, start_pose, offset=0.2, axis='z', gain=0.6, max_joint_vel=0.5):
        super().__init__('vel_trajectory_controller')
        
        self.robot = RobotKinematics()
        self.offset = offset
        self.gain = gain
        self.max_joint_vel = max_joint_vel
        self.axis_idx = {'x': 0, 'y': 1, 'z': 2}.get(axis.lower(), 2)
        self.get_logger().info("Started")

        # Limits
        bounds = np.array(self.robot.joint_bounds)
        self.joint_low, self.joint_high = bounds[:, 0], bounds[:, 1]
        
        # State
        self.current_q = None
        self.start_pose = np.array(start_pose, dtype=float)
        self.direction = 1
        self.initialized = False

        self.target_start_q = None  # unused in velocity mode; kept for reference

        # ROS
        self._joint_cmd_pub = self.create_publisher(JointTrajectory, 'joint_cmds', 10)
        self._sub = self.create_subscription(JointState, 'joint_states', self._on_joint_states, qos_profile_sensor_data)

        self.create_timer(0.04, self.control_loop)
        # Defer heavy IK computation until after spin() starts so RViz keeps updating
        self._setup_timer = self.create_timer(0.1, self._setup_once)

    def _setup_once(self):
        """Fires once after spin() starts, then launches IK in a background thread."""
        self._setup_timer.cancel()
        threading.Thread(target=self._run_setup, daemon=True).start()

    def _run_setup(self):
        """Background thread: validates workspace and computes start IK without blocking spin()."""
        try:
            if not self._validate_offset():
                self.get_logger().error("Robot cannot reach any part of the requested trajectory.")
                return

            ik_solutions = self.robot.inverse_kinematics(self.start_pose[:3])
            if not ik_solutions:
                self.get_logger().error(f"IK found no solution for start_pose={self.start_pose[:3]}")
                return

            self.target_start_q = np.array(ik_solutions[0])
            self.get_logger().info(f"Setup done. offset={self.offset:.2f} m, target_q={np.round(self.target_start_q, 3)}")
        except Exception as e:
            self.get_logger().error(f"Setup failed with exception: {e}")

    def _validate_offset(self):
        """Shrinks offset until the oscillation is within reach."""
        while self.offset > 0.02:
            pts = [self.start_pose[:3].copy(), self.start_pose[:3].copy()]
            pts[0][self.axis_idx] += self.offset
            pts[1][self.axis_idx] -= self.offset
            self.get_logger().info(f"offset = {self.offset}")
            if self.robot.inverse_kinematics(pts[0]) and self.robot.inverse_kinematics(pts[1]):
                return True
            self.offset -= 0.02
        return False

    def _on_joint_states(self, msg: JointState):
        q = np.array(msg.position[:5])
        self.current_q = np.clip(q, self.joint_low, self.joint_high)

    def control_loop(self):
        if self.target_start_q is None:
            self.get_logger().info('Waiting for IK setup...', throttle_duration_sec=2.0)
            return
        if self.current_q is None:
            self.get_logger().warn('Waiting for joint_states — check QoS or simulator.', throttle_duration_sec=2.0)
            return

        # Latch start pose to actual robot position on first tick
        if not self.initialized:
            cur_pose = self.robot.forward_kinematics(self.current_q)
            self.start_pose[:3] = cur_pose[:3]
            self.target_pose = self.start_pose.copy() if hasattr(self, 'target_pose') else self.start_pose.copy()
            self.initialized = True
            self.get_logger().info(f'Latched start pose: {np.round(self.start_pose[:3], 3)}')
            return

        cur_pose = self.robot.forward_kinematics(self.current_q)

        # Distance remaining to current target
        target_axis = self.start_pose[self.axis_idx] + self.direction * self.offset
        remaining = abs(target_axis - cur_pose[self.axis_idx])

        # Switch direction when within 2 cm of target
        if remaining < 0.02:
            self.direction *= -1

        # Proportional speed: full gain when far, decelerate in last 5 cm
        decel_dist = 0.05
        speed = self.gain if remaining > decel_dist else self.gain * (remaining / decel_dist)

        v_cartesian = np.zeros(6)
        v_cartesian[self.axis_idx] = self.direction * speed

        # Correct drift on the two uncontrolled position axes (x, y)
        for i in range(3):
            if i != self.axis_idx:
                v_cartesian[i] = self.gain * (self.start_pose[i] - cur_pose[i])

        # Damped Jacobian Inverse
        jac = self.robot.jacobian(self.current_q)
        inv_jac = jac.T @ np.linalg.inv(jac @ jac.T + (0.02**2) * np.eye(6))
        q_dot = inv_jac @ v_cartesian

        # Bound Enforcement
        buf, stop = 0.08, 0.01
        for i in range(5):
            d_hi, d_lo = self.joint_high[i] - self.current_q[i], self.current_q[i] - self.joint_low[i]
            if q_dot[i] > 0:
                if d_hi < stop: q_dot[i] = 0.0
                elif d_hi < buf: q_dot[i] *= (d_hi / buf)
            elif q_dot[i] < 0:
                if d_lo < stop: q_dot[i] = 0.0
                elif d_lo < buf: q_dot[i] *= (d_lo / buf)

        q_dot = np.clip(q_dot, -self.max_joint_vel, self.max_joint_vel)
        self._publish_velocity(q_dot)

    def _publish_velocity(self, q_dot):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.robot.joint_names
        p = JointTrajectoryPoint()
        p.velocities = list(q_dot) + [0.0]
        msg.points = [p]
        self._joint_cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    # Start at 20cm height, try to oscillate 15cm up/down
    node = VelTrajectoryController(start_pose=(0.0, 0.4, 0.2, 0,0,0), offset=0.15)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()