import threading
import time
import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

JOINT_NAMES = [
    'Shoulder_Rotation',
    'Shoulder_Pitch',
    'Elbow',
    'Wrist_Pitch',
    'Wrist_Roll',
    'Gripper',
]

JOINT_LIMITS_DEG = [
    (-115.0, 115.0),   # Shoulder_Rotation
    ( -90.0,  90.0),   # Shoulder_Pitch
    ( -90.0,  90.0),   # Elbow
    ( -90.0,  90.0),   # Wrist_Pitch
    (-180.0, 180.0),   # Wrist_Roll
    (   0.0,  90.0),   # Gripper  (0=closed, 90=open)
]
    
HOME_DEG = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


class SetJointPosition(Node):

    def __init__(self):
        super().__init__('set_joint_position')
        self._target = list(np.deg2rad(HOME_DEG))
        self._publisher = self.create_publisher(JointTrajectory, 'joint_cmds', 10)
        self.create_timer(0.1, self._publish)

    def _publish(self):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = list(self._target)
        msg.points = [pt]
        self._publisher.publish(msg)

    def set_target(self, positions_rad):
        self._target = list(positions_rad)


class JointStateSetter:
    """Publishes joint positions to ``joint_states`` (robot_state_publisher / RViz).

    This is the approach required when only ``robot_state_publisher`` is running
    (e.g. ``ros2 launch lerobot rviz.launch.py``), because there is no simulation
    node to consume ``joint_cmds``.

    Usage (external)::

        rclpy.init()
        node = rclpy.create_node('my_node')
        setter = JointStateSetter(node)
        setter.set([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])   # 6 joints, radians
        node.destroy_node()
        rclpy.shutdown()
    """

    def __init__(self, node: Node):
        self._node = node
        self._pub = node.create_publisher(JointState, 'joint_states', 10)

    def set(self, positions_rad, joint_names=None):
        """Publish *positions_rad* to ``joint_states`` and wait for TF to settle.

        Parameters
        ----------
        positions_rad : sequence of float
            Joint angles in radians (all 6 joints, including Gripper).
        joint_names : list[str] or None
            Override joint names; defaults to the module-level ``JOINT_NAMES``.
        """
        names = joint_names if joint_names is not None else JOINT_NAMES
        msg = JointState()
        msg.name = list(names)
        msg.position = [float(v) for v in positions_rad]
        for _ in range(10):
            msg.header.stamp = self._node.get_clock().now().to_msg()
            self._pub.publish(msg)
            rclpy.spin_once(self._node, timeout_sec=0.05)
        # Spin (not sleep) so TF callbacks are processed while waiting for
        # robot_state_publisher to broadcast the updated transforms.
        for _ in range(10):
            rclpy.spin_once(self._node, timeout_sec=0.05)


def _print_header():
    print('\n' + '=' * 58)
    print('  LeRobot — Interactive Joint Position Controller')
    print('=' * 58)
    print(f'  {"Joint":<22} {"Min":>6}  {"Max":>6}  {"Unit"}')
    print(f'  {"-"*22}  {"-"*6}  {"-"*6}  {"-"*4}')
    for name, (lo, hi) in zip(JOINT_NAMES, JOINT_LIMITS_DEG):
        print(f'  {name:<22} {lo:>6.1f}  {hi:>6.1f}  deg')
    print('=' * 58)
    print('  Enter 6 space-separated values in degrees.')
    print('  Press Enter with no input to resend current target.')
    print('  Type  "home"  to go to the home position.')
    print('  Type  "quit"  to exit.')
    print('=' * 58)


def _format_target(positions_rad):
    vals = '  '.join(f'{np.rad2deg(v):7.2f}°' for v in positions_rad)
    return f'  Current target: [ {vals} ]'


def _input_loop(node: SetJointPosition):
    _print_header()
    print(_format_target(node._target))

    while rclpy.ok():
        try:
            raw = input('\n  > ').strip()
        except (EOFError, KeyboardInterrupt):
            break

        if not raw:
            print('  Resending current target.')
            print(_format_target(node._target))
            continue

        if raw.lower() == 'exit':
            print('  Shutting down.')
            rclpy.shutdown()
            break

        if raw.lower() == 'home':
            positions_rad = list(np.deg2rad(HOME_DEG))
            node.set_target(positions_rad)
            print('  Moved to home position.')
            print(_format_target(node._target))
            continue

        parts = raw.split()
        if len(parts) != 6:
            print(f'  Error: expected 6 values, got {len(parts)}. Try again.')
            continue

        try:
            values_deg = [float(p) for p in parts]
        except ValueError:
            print('  Error: non-numeric input detected. Try again.')
            continue

        # Validate limits
        errors = []
        for i, (val, (lo, hi)) in enumerate(zip(values_deg, JOINT_LIMITS_DEG)):
            if not (lo <= val <= hi):
                errors.append(f'    {JOINT_NAMES[i]}: {val:.1f}° is outside [{lo:.1f}°, {hi:.1f}°]')
        if errors:
            print('  Error: values out of range:')
            for e in errors:
                print(e)
            continue

        positions_rad = list(np.deg2rad(values_deg))
        node.set_target(positions_rad)
        print('  Command sent.')
        print(_format_target(node._target))


def main(args=None):
    rclpy.init(args=args)
    node = SetJointPosition()

    # Run terminal input on a background thread so rclpy.spin() is not blocked
    input_thread = threading.Thread(target=_input_loop, args=(node,), daemon=True)
    input_thread.start()

    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()
