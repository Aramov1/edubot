import rclpy
import numpy as np
from rclpy.node import Node

from geometry_msgs.msg import Pose

from python_controllers import curve_definition as cd


class EEPosePublisher(Node):
    """Publish a time-parameterized EE pose along a curve in the x = X_PLANE plane.

    The curve shape, plane position, EE orientation and traversal speed are
    all defined in curve_definition.py.

    Published topic: target_EE_pose  (geometry_msgs/Pose)
    """

    def __init__(self):
        super().__init__('EE_pose_publisher')

        self._publisher = self.create_publisher(Pose, 'target_EE_pose', 10)
        self._t0 = self.get_clock().now()

        # Publish at 25 Hz
        self.create_timer(0.04, self._timer_cb)

        self.get_logger().info(
            f'Publishing curve at x={cd.X_PLANE:.3f} m, '
            f'period={cd.PERIOD:.1f} s')

    def _timer_cb(self):
        now  = self.get_clock().now()
        dt   = (now - self._t0).nanoseconds * 1e-9

        # Normalised parameter t ∈ [0, 1), cycling with PERIOD
        t = (dt % cd.PERIOD) / cd.PERIOD

        y, z = cd.curve(t)

        msg = Pose()
        msg.position.x = cd.X_PLANE
        msg.position.y = float(y)
        msg.position.z = float(z)

        qx, qy, qz, qw = cd.EE_ORIENTATION
        msg.orientation.x = float(qx)
        msg.orientation.y = float(qy)
        msg.orientation.z = float(qz)
        msg.orientation.w = float(qw)

        self._publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EEPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
