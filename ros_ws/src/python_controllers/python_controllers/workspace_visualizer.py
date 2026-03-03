import sys

# RobotKinematics lives outside the ROS workspace — add it to the path
sys.path.insert(0, '/home/andre/TU_Delft/edubot/assignment')

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration

from robot_kinematics import RobotKinematics


class WorkspaceVisualizer(Node):
    """
    Computes the full reachable workspace of the SO-ARM100 using RobotKinematics
    and publishes it as a POINTS Marker to /fk_ee so it appears in RViz.

    The workspace is computed once on startup and re-published every 5 s so that
    RViz instances that start after this node still receive the data (transient-local
    QoS also covers that case, but the periodic republish is a belt-and-suspenders
    safety net).
    """

    def __init__(self):
        super().__init__('workspace_visualizer')

        # Transient-local so late-joining subscribers (e.g. RViz) receive the marker
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._pub = self.create_publisher(Marker, '/fk_ee', qos)

        self.get_logger().info('Building kinematics model and computing workspace — this takes a few seconds...')

        kinematics = RobotKinematics()
        x, y, z = kinematics.compute_workspace()
        n_pts = x.flatten().shape[0]

        self.get_logger().info(f'Workspace computed: {n_pts} points. Building marker...')
        self._marker = self._build_marker(x.flatten(), y.flatten(), z.flatten())
        self.get_logger().info('Publishing workspace to /fk_ee (visible in RViz under Marker display).')

        self._publish()
        self._timer = self.create_timer(5.0, self._publish)

    # ------------------------------------------------------------------

    def _build_marker(self, x: np.ndarray, y: np.ndarray, z: np.ndarray) -> Marker:
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.ns = 'workspace'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD

        # Point size in metres — small enough to avoid overlap at 12-pt resolution
        marker.scale.x = 0.004
        marker.scale.y = 0.004

        # Cyan, semi-transparent so the robot model stays visible underneath
        marker.color.r = 0.0
        marker.color.g = 0.75
        marker.color.b = 1.0
        marker.color.a = 0.35

        # lifetime = 0 means the marker never expires
        marker.lifetime = Duration(sec=0, nanosec=0)

        marker.points = [
            Point(x=float(xi), y=float(yi), z=float(zi))
            for xi, yi, zi in zip(x, y, z)
        ]

        return marker

    def _publish(self):
        self._marker.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(self._marker)


# ----------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = WorkspaceVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()