import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros


def quat_to_euler_zxy(x, y, z, w):
    """Convert quaternion to pitch (X), roll (Y), yaw (Z) using ZXY convention.

    Intrinsic rotation order: Z (yaw) → X (pitch) → Y (roll).
    Valid when pitch ∈ (−90°, 90°).

    Returns (pitch_deg, roll_deg, yaw_deg).
    """
    r20 = 2.0 * (x * z - w * y)
    r21 = 2.0 * (y * z + w * x)
    r22 = 1.0 - 2.0 * (x * x + y * y)
    r01 = 2.0 * (x * y - w * z)
    r11 = 1.0 - 2.0 * (x * x + z * z)

    pitch = np.arctan2(r21, np.sqrt(r20**2 + r22**2))    # about X
    roll  = np.arctan2(-r20, r22)                        # about Y
    yaw   = np.arctan2(-r01, r11)                        # about Z

    return np.rad2deg(pitch), np.rad2deg(roll), np.rad2deg(yaw)


class EEPositionReader:
    """Reads the ``world → gripper_center`` TF and returns the EE pose.

    Usage (external)::

        rclpy.init()
        node = rclpy.create_node('my_node')
        reader = EEPositionReader(node)
        pose = reader.read()   # [x, y, z, pitch_deg, roll_deg] or None
        node.destroy_node()
        rclpy.shutdown()

    The returned list matches the FK output order: ``[x, y, z, pitch, roll]``
    where pitch and roll are in degrees.
    """

    def __init__(self, node: Node):
        self._node = node
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, node)

    def read(self, retries: int = 20):
        """Return ``[x, y, z, pitch_deg, roll_deg]`` or ``None`` on failure.

        Parameters
        ----------
        retries : int
            Number of spin attempts before giving up (each ~0.1 s).
        """
        for attempt in range(retries):
            rclpy.spin_once(self._node, timeout_sec=0.1)
            try:
                tf_stamped = self._tf_buffer.lookup_transform(
                    'world', 'gripper_center', Time())
                pos = tf_stamped.transform.translation
                rot = tf_stamped.transform.rotation
                pitch_deg, roll_deg, yaw_deg = quat_to_euler_zxy(
                    rot.x, rot.y, rot.z, rot.w)
                return [pos.x, pos.y, pos.z, pitch_deg, roll_deg, yaw_deg]
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                if attempt == retries - 1:
                    return None


class ReadEEPosition(Node):
    """Standalone ROS2 node that continuously prints the EE pose."""

    def __init__(self):
        super().__init__('read_ee_position')
        self._reader = EEPositionReader(self)
        self.create_timer(0.1, self._timer_callback)

    def _timer_callback(self):
        pose = self._reader.read(retries=1)
        if pose is None:
            self.get_logger().warn(
                'TF not available yet — is robot_state_publisher running?',
                throttle_duration_sec=2.0)
            return
        x, y, z, pitch, roll = pose
        self.get_logger().info(
            f'EE Position (m): x={x:+.4f}  y={y:+.4f}  z={z:+.4f} | '
            f'Orientation (deg): pitch={pitch:+7.2f}  roll={roll:+7.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = ReadEEPosition()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
