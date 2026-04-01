import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import tf2_ros
from scipy.spatial.transform import Rotation as SciPyRotation

# Import the pre-configured Sensor Data QoS profile
from rclpy.qos import qos_profile_sensor_data

# Import your kinematics solver
from .robot_kinematics import RobotKinematics


class PublishEEPose(Node):
    """
    Publishes the end-effector pose (position + orientation) to the /ee_pose topic.
    Reads the EE position from TF (world -> gripper_center) and republishes it
    as a PoseStamped message for easier consumption by other nodes.
    Falls back to calculating FK via RobotKinematics if TF is unavailable.
    
    Usage:
        ros2 run python_controllers publish_ee_pose
    """

    def __init__(self):
        super().__init__('publish_ee_pose')

        # TF2 setup
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Initialize the Kinematics solver
        self._kinematics = RobotKinematics()

        # Joint state variables initialization
        self.current_q_arm = None
        self.current_q_gripper = None

        # Publisher for EE pose
        self._ee_pose_pub = self.create_publisher(PoseStamped, 'ee_pose', 10)

        # Subscriber for joint states (needed for FK fallback)
        # FIXED: Added qos_profile_sensor_data to fix the Incompatible QoS error
        self._joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self._joint_state_callback, qos_profile_sensor_data)

        # Timer to periodically read and publish EE pose
        self._timer = self.create_timer(0.1, self._timer_callback)
        
        self.get_logger().info('Publishing EE pose to /ee_pose topic')

    def _joint_state_callback(self, msg):
        """Read arm and gripper separately and store them."""
        # Assuming the first 5 joints belong to the arm
        self.current_q_arm = np.array(msg.position[:5])
        if len(msg.position) > 5:
            self.current_q_gripper = msg.position[5]

    def _timer_callback(self):
        """Periodically read EE pose from TF or fallback to FK and publish it."""
        try:
            # Try to get the pose from TF first
            tf_stamped = self._tf_buffer.lookup_transform('world', 'gripper_center', Time())
            
            # Extract position
            pos_x = tf_stamped.transform.translation.x
            pos_y = tf_stamped.transform.translation.y
            pos_z = tf_stamped.transform.translation.z
            
            # Extract rotation
            rot_x = tf_stamped.transform.rotation.x
            rot_y = tf_stamped.transform.rotation.y
            rot_z = tf_stamped.transform.rotation.z
            rot_w = tf_stamped.transform.rotation.w
            
            # Use the TF timestamp
            stamp = tf_stamped.header.stamp

        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            
            # FIXED: Use once=True instead of throttle_duration_sec
            self.get_logger().warn(
                'TF not available — falling back to RobotKinematics forward_kinematics',
                once=True)
            
            if self.current_q_arm is None:
                # FIXED: Use once=True instead of throttle_duration_sec
                self.get_logger().warn(
                    'Waiting for /joint_states to compute FK fallback...',
                    once=True)
                return
            
            try:
                # FALLBACK: Use forward kinematics
                fk_result = self._kinematics.forward_kinematics(self.current_q_arm)
                
                # Extract position from FK (first 3 elements)
                pos_x, pos_y, pos_z = float(fk_result[0]), float(fk_result[1]), float(fk_result[2])
                
                # Extract Euler angles (last 3 elements)
                rx, ry, rz = float(fk_result[3]), float(fk_result[4]), float(fk_result[5])
                
                # Convert Euler (Extrinsic XYZ) to Quaternion using SciPy
                quat = SciPyRotation.from_euler('xyz', [rx, ry, rz]).as_quat()
                rot_x, rot_y, rot_z, rot_w = float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])
                
                # Use the current ROS time since we aren't getting a timestamp from TF
                stamp = self.get_clock().now().to_msg()
                
            except Exception as e:
                self.get_logger().error(f'FK fallback failed: {e}', throttle_duration_sec=2.0)
                return

        # Build PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = 'world'
        
        # Position
        pose_msg.pose.position.x = pos_x
        pose_msg.pose.position.y = pos_y
        pose_msg.pose.position.z = pos_z
        
        # Orientation 
        pose_msg.pose.orientation.x = rot_x
        pose_msg.pose.orientation.y = rot_y
        pose_msg.pose.orientation.z = rot_z
        pose_msg.pose.orientation.w = rot_w
        
        # Publish
        self._ee_pose_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PublishEEPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()