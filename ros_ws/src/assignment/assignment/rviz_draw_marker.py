import time
import rclpy
from rclpy.node import Node

import tf2_ros

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, DurabilityPolicy

class RvizDrawMarker(Node):
    def __init__(self, verbose = False):
        super().__init__('rviz_draw_marker')
        self.verbose = verbose

        # TF2 Setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_found = False

        # Keep dots visible in RVIZ after node is destroyed.
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(Marker, '/workspace_markers', qos)

        # Timer to wait for TF to connect
        self.tf_timer = self.create_timer(0.5, self.check_tf_ready)
        self.get_logger().info("Draw Marker Ready Started. Waiting for TF...")

    def check_tf_ready(self):
        rclpy.spin_once(self, timeout_sec=0.1)

        try:
            self.tf_buffer.lookup_transform('world', 'gripper_center', rclpy.time.Time())
            self.get_logger().info("TF Connection Established!")
            self.tf_found = True
            self.tf_timer.destroy()

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn("Waiting for TF connection ...", throttle_duration_sec=2.0)
            return


    def publish_workspace(self, x, y, z):
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'workspace'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD

        # Appearance
        marker.scale.x = marker.scale.y = 0.005
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 1.0, 0.5, 0.4
        
        # Load Points
        marker.points = [Point(x=float(xi), y=float(yi), z=float(zi)) for xi, yi, zi in zip(x, y, z)]
        
        # Publish multiple times to ensure the middleware catches it before we exit
        self.pub.publish(marker)
        time.sleep(5.0) # Give DDS time to breathe
        
        if self.verbose:
            self.get_logger().info(f"Published {len(marker.points)} points to RViz.")