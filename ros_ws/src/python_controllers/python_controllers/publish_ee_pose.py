import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
import tf2_ros


class PublishEEPose(Node):
    """
    Publishes the end-effector pose (position + orientation) to the /ee_pose topic.
    Reads the EE position from TF (world -> gripper_center) and republishes it
    as a PoseStamped message for easier consumption by other nodes.
    
    Usage:
        ros2 run python_controllers publish_ee_pose
    """

    def __init__(self):
        super().__init__('publish_ee_pose')

        # TF2 setup
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Publisher for EE pose
        self._ee_pose_pub = self.create_publisher(PoseStamped, 'ee_pose', 10)

        # Timer to periodically read and publish EE pose
        self._timer = self.create_timer(0.1, self._timer_callback)
        
        self.get_logger().info('Publishing EE pose to /ee_pose topic')

    def _timer_callback(self):
        """Periodically read EE pose from TF and publish it."""
        try:
            tf_stamped = self._tf_buffer.lookup_transform('world', 'gripper_center', Time())
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            self.get_logger().warn(
                'TF not available yet — is robot_state_publisher running?',
                throttle_duration_sec=2.0)
            return

        # Extract position
        pos = tf_stamped.transform.translation
        
        # Extract rotation (already a quaternion from TF)
        rot = tf_stamped.transform.rotation

        # Build PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = tf_stamped.header.stamp
        pose_msg.header.frame_id = 'world'
        
        # Position
        pose_msg.pose.position.x = pos.x
        pose_msg.pose.position.y = pos.y
        pose_msg.pose.position.z = pos.z
        
        # Orientation (quaternion from TF)
        pose_msg.pose.orientation.x = rot.x
        pose_msg.pose.orientation.y = rot.y
        pose_msg.pose.orientation.z = rot.z
        pose_msg.pose.orientation.w = rot.w
        
        # Publish
        self._ee_pose_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PublishEEPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
