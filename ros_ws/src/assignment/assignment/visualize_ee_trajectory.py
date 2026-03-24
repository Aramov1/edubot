import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy


class VisualizeEETrajectory(Node):
    """
    Visualizes the end-effector trajectory in RViz by subscribing to /ee_pose
    and drawing a LINE_STRIP marker of the trajectory path.
    
    This node is independent and won't interfere with other functionality.
    
    Usage:
        ros2 run python_controllers visualize_ee_trajectory
    """

    def __init__(self):
        super().__init__('visualize_ee_trajectory')

        # Distance threshold (in meters) to detect a jump and reset the line.
        # 0.001 means if the EE moves more than 1 cm in one reading, it breaks the line.
        self.jump_threshold = 0.05 

        # QoS profile for marker publishing
        qos_marker = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        # QoS profile for subscribing to ee_pose
        qos_sub = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        # Subscribe to EE pose from publish_ee_pose.py
        self._ee_pose_sub = self.create_subscription(
            PoseStamped,
            'ee_pose',
            self.ee_pose_callback,
            qos_sub
        )

        # Publisher for trajectory marker
        self._marker_pub = self.create_publisher(Marker, 'ee_trajectory_marker', qos_marker)

        # Store trajectory points
        self.trajectory_points = []
        self.marker_id = 0

        # Timer to periodically publish the marker
        self._timer = self.create_timer(0.5, self.publish_trajectory_marker)

        self.get_logger().info('Visualizing EE trajectory. Publish marker to /ee_trajectory_marker')

    def ee_pose_callback(self, msg: PoseStamped):
        """Receive EE pose, check for jumps, and accumulate trajectory points."""
        new_point = Point()
        new_point.x = msg.pose.position.x
        new_point.y = msg.pose.position.y
        new_point.z = msg.pose.position.z
        
        # Check distance if we already have points
        if len(self.trajectory_points) > 0:
            last_point = self.trajectory_points[-1]
            distance = math.sqrt(
                (new_point.x - last_point.x)**2 +
                (new_point.y - last_point.y)**2 +
                (new_point.z - last_point.z)**2
            )
            
            # If the distance is larger than our threshold, clear the line!
            if distance > self.jump_threshold:
                self.get_logger().info(f"Jump detected ({distance:.3f}m > {self.jump_threshold}m). Resetting trajectory drawing.")
                self.trajectory_points.clear()
        
        self.trajectory_points.append(new_point)

    def publish_trajectory_marker(self):
        """Publish trajectory as a LINE_STRIP marker."""
        if len(self.trajectory_points) < 2:
            return  # Need at least 2 points to draw a line

        # Create LINE_STRIP marker
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'ee_trajectory'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Line properties
        marker.scale.x = 0.01  # Line width
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8

        # Add all trajectory points
        marker.points = self.trajectory_points

        # Lifetime = 0 means never expires
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        self._marker_pub.publish(marker)

    def reset_trajectory(self):
        """Reset the accumulated trajectory."""
        self.trajectory_points.clear()
        self.get_logger().info('Trajectory reset manually')


def main(args=None):
    rclpy.init(args=args)
    node = VisualizeEETrajectory()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()