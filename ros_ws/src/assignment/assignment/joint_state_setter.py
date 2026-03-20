import threading
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINT_NAMES = [
    'Shoulder_Rotation',
    'Shoulder_Pitch',
    'Elbow',
    'Wrist_Pitch',
    'Wrist_Roll',
    'Gripper',
]

JOINT_LIMITS_RAD = [
    (np.deg2rad(-115.0), np.deg2rad(115.0)),   # Shoulder_Rotation
    (np.deg2rad( -90.0), np.deg2rad( 90.0)),   # Shoulder_Pitch
    (np.deg2rad( -90.0), np.deg2rad( 90.0)),   # Elbow
    (np.deg2rad( -90.0), np.deg2rad( 90.0)),   # Wrist_Pitch
    (np.deg2rad(-180.0), np.deg2rad(180.0)),   # Wrist_Roll
    (np.deg2rad(   0.0), np.deg2rad( 90.0)),   # Gripper  (0=closed, pi/2=open)
]

HOME_RAD = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


class SetJointPosition(Node):
    """Continuously publishes joint positions to ``joint_states`` at 10 Hz.

    Publishes JointState (not JointTrajectory) so that robot_state_publisher
    picks it up and RViz moves without requiring a simulation node.
    """

    def __init__(self):
        super().__init__('set_joint_position')
        self._target = list(HOME_RAD)
        # Publish to joint_states so robot_state_publisher / RViz reacts
        self._publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.create_timer(0.1, self._publish)

    def _publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = list(self._target)
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
    print('\n' + '=' * 62)
    print('  LeRobot — Interactive Joint Position Controller')
    print('=' * 62)
    print(f'  {"Joint":<22} {"Min (rad)":>9}  {"Max (rad)":>9}')
    print(f'  {"-"*22}  {"-"*9}  {"-"*9}')
    for name, (lo_rad, hi_rad) in zip(JOINT_NAMES, JOINT_LIMITS_RAD):
        print(f'  {name:<22} {lo_rad:>9.4f}  {hi_rad:>9.4f}')
    print('=' * 62)
    print('  Enter 6 space-separated values in radians.')
    print('  Press Enter with no input to resend current target.')
    print('  Type  "home"  to go to the home position.')
    print('  Type  "exit"  to quit.')
    print('=' * 62)


def _format_target(positions_rad):
    vals = '  '.join(f'{v:7.4f}' for v in positions_rad)
    return f'  Current target (rad): [ {vals} ]'


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
            positions_rad = list(HOME_RAD)
            node.set_target(positions_rad)
            print('  Moved to home position.')
            print(_format_target(node._target))
            continue

        parts = raw.split()
        if len(parts) != 6:
            print(f'  Error: expected 6 values, got {len(parts)}. Try again.')
            continue

        try:
            values_rad = [float(p) for p in parts]
        except ValueError:
            print('  Error: non-numeric input detected. Try again.')
            continue

        # Validate limits in radians
        errors = []
        for i, (val_rad, (lo_rad, hi_rad)) in enumerate(zip(values_rad, JOINT_LIMITS_RAD)):
            if not (lo_rad <= val_rad <= hi_rad):
                errors.append(
                    f'    {JOINT_NAMES[i]}: {val_rad:.4f} rad is outside '
                    f'[{lo_rad:.4f}, {hi_rad:.4f}] rad'
                )
        if errors:
            print('  Error: values out of range:')
            for e in errors:
                print(e)
            continue

        node.set_target(values_rad)
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
