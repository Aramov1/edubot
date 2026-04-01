import os
import sys
_ASSIGNMENT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_PROJECT_ROOT   = os.path.dirname(_ASSIGNMENT_DIR)
sys.path.insert(0, _ASSIGNMENT_DIR)
sys.path.insert(0, os.path.join(_PROJECT_ROOT, 'ros_ws', 'src', 'assignment', 'assignment'))

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

import numpy as np
import tf2_ros
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from assignment.robot_kinematics import RobotKinematics


class WorkspaceVisualizer(Node):
    def __init__(self, res=15):
        super().__init__('workspace_visualizer')
        self.res = res
        self.robot = RobotKinematics()

        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Latched publisher so points stay visible after node exits
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(Marker, '/workspace_markers', qos)

        self.tf_timer = self.create_timer(0.5, self._check_tf_ready)
        self.get_logger().info("WorkspaceVisualizer started. Waiting for TF...")

    def _check_tf_ready(self):
        try:
            self.tf_buffer.lookup_transform('world', 'gripper_center', rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn("Waiting for TF connection...", throttle_duration_sec=2.0)
            return

        self.tf_timer.destroy()
        self.get_logger().info("TF ready. Computing workspace...")
        self._compute_and_publish()

    def _compute_and_publish(self):
        spaces = [np.linspace(b[0], b[1], self.res) for b in self.robot.joint_bounds]
        grids = np.meshgrid(*spaces, indexing='ij')
        q_flat = [g.flatten() for g in grids]

        pose = self.robot.forward_kinematics(q_flat)
        x_pts, y_pts, z_pts = pose[0], pose[1], pose[2]

        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'workspace'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = marker.scale.y = 0.005
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 1.0 , 0.5, 0.1
        marker.points = [
            Point(x=float(xi), y=float(yi), z=float(zi))
            for xi, yi, zi in zip(x_pts, y_pts, z_pts)
        ]

        self.pub.publish(marker)
        time.sleep(5.0)  # Give DDS time to deliver before node exits
        self.get_logger().info(f"Published {len(marker.points)} workspace points to RViz.")


def main():
    print(f"\n{'='*60}\n WORKSPACE VISUALIZER START\n{'='*60}")
    rclpy.init()
    node = WorkspaceVisualizer(res=15)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
