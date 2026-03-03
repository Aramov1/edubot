import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros


def quat_to_euler_deg(x, y, z, w):
    """Convert quaternion to roll, pitch, yaw in degrees."""
    # Roll (rotation around X)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (rotation around Y)
    sinp = 2.0 * (w * y - z * x)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)

    # Yaw (rotation around Z)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return np.rad2deg(roll), np.rad2deg(pitch), np.rad2deg(yaw)


class ReadEEPose(Node):

    def __init__(self):
        super().__init__('read_ee_pose')

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._timer = self.create_timer(0.1, self._timer_callback)

    def _timer_callback(self):
        try:
            t = self._tf_buffer.lookup_transform('world', 'gripper_center', Time())
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            self.get_logger().warn('TF not available yet — is robot_state_publisher running?',
                                   throttle_duration_sec=2.0)
            return

        pos = t.transform.translation
        rot = t.transform.rotation

        roll, pitch, yaw = quat_to_euler_deg(rot.x, rot.y, rot.z, rot.w)

        self.get_logger().info(
            f'\nEE Position  (m)  : x={pos.x:+.4f}  y={pos.y:+.4f}  z={pos.z:+.4f} | '
            f'Orientation (deg) : roll={roll:+7.2f}  pitch={pitch:+7.2f}  yaw={yaw:+7.2f}\ '
        )


def main(args=None):
    rclpy.init(args=args)

    read_ee_pose = ReadEEPose()

    rclpy.spin(read_ee_pose)

    read_ee_pose.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
