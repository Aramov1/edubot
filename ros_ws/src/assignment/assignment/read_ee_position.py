import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros


# R_home.T — rotates out the fixed structural offset so home joints → (0, 0, 0)
_R_HOME_INV = np.array([[1, 0,  0],
                         [0, 0, -1],
                         [0, 1,  0]])


def quat_to_euler_zxy(x, y, z, w):
    """Convert quaternion to ZXY Euler angles relative to home orientation.

    Angles are referenced to the home configuration (all joints = 0),
    so pitch=roll=yaw=0 when the arm is at home.
    Intrinsic rotation order: Z (yaw) → X (pitch) → Y (roll).
    Valid when pitch ∈ (−π/2, π/2).

    Returns (pitch_rad, roll_rad, yaw_rad).
    """
    R = np.array([
        [1 - 2*(y*y + z*z),  2*(x*y - w*z),      2*(x*z + w*y)],
        [    2*(x*y + w*z),  1 - 2*(x*x + z*z),  2*(y*z - w*x)],
        [    2*(x*z - w*y),      2*(y*z + w*x),  1 - 2*(x*x + y*y)],
    ])
    R_rel = _R_HOME_INV @ R
    pitch = np.arcsin(R_rel[2, 1])
    roll  = np.arctan2(-R_rel[2, 0], R_rel[2, 2])
    yaw   = np.arctan2(-R_rel[0, 1], R_rel[1, 1])
    return pitch, roll, yaw


class EEPositionReader:
    """
    Reads the ``world → gripper_center`` TF and returns the EE pose.
    The returned list matches the FK output order: ``[x, y, z, pitch, roll, yaw]``
    all position is in metres and all angles are in radians.

    Usage (external)::

        rclpy.init()
        node = rclpy.create_node('my_node')
        reader = EEPositionReader(node)
        pose = reader.read()   # [x, y, z, pitch, roll, yaw] in metres/radians, or None
        node.destroy_node()
        rclpy.shutdown()
    """

    def __init__(self, node: Node):
        self._node = node
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, node)

    def lookup(self):
        """Direct TF lookup — no spin_once.

        Use this inside a node that is already running under ``rclpy.spin()``.
        The outer spin drives the TF listener; calling spin_once here would be
        reentrant and the SingleThreadedExecutor would silently skip it.

        Returns ``[x, y, z, pitch, roll, yaw]`` (metres, radians) or ``None``.
        """
        try:
            tf_stamped = self._tf_buffer.lookup_transform(
                'world', 'gripper_center', Time())
            pos = tf_stamped.transform.translation
            rot = tf_stamped.transform.rotation
            pitch, roll, yaw = quat_to_euler_zxy(rot.x, rot.y, rot.z, rot.w)
            return [pos.x, pos.y, pos.z, pitch, roll, yaw]
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None

    def read(self, retries: int = 20):
        """Return ``[x, y, z, pitch, roll, yaw]`` (metres, radians) or ``None`` on failure.

        Drives its own spin loop — use this outside of ``rclpy.spin()``,
        e.g. in a script that calls read() once per IK solution.

        retries : Number of spin attempts before giving up (each ~0.1 s).
        """
        for attempt in range(retries):
            rclpy.spin_once(self._node, timeout_sec=0.1)
            result = self.lookup()
            if result is not None:
                return result
            if attempt == retries - 1:
                return None


class ReadEEPosition(Node):
    """Standalone ROS2 node that continuously prints the EE pose."""

    def __init__(self):
        super().__init__('read_ee_position')
        self._reader = EEPositionReader(self)
        self.create_timer(0.1, self._timer_callback)

    def _timer_callback(self):
        # Use lookup() not read(): we are already inside rclpy.spin() so the
        # TF buffer is being updated by the outer executor — no spin_once needed.
        pose = self._reader.lookup()
        if pose is None:
            self.get_logger().warn(
                'TF not available yet — is robot_state_publisher running?',
                throttle_duration_sec=2.0)
            return

        x, y, z, pitch, roll, yaw = pose
        self.get_logger().info(
            f'EE Position (m): x={x:+.4f}  y={y:+.4f}  z={z:+.4f} | '
            f'Orientation (rad): pitch={pitch:+.4f}  roll={roll:+.4f}  yaw={yaw:+.4f}')


def main(args=None):
    rclpy.init(args=args)
    node = ReadEEPosition()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
