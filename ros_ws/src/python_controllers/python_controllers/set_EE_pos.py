import rclpy
import numpy as np
from rclpy.node import Node
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState


# ── Forward kinematics ────────────────────────────────────────────────────────

def _make_T(xyz, rpy):
    """4x4 homogeneous transform from URDF joint origin (xyz, roll-pitch-yaw).

    URDF convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    """
    x, y, z = xyz
    roll, pitch, yaw = rpy
    cr, sr = np.cos(roll),  np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw),   np.sin(yaw)
    R = np.array([
        [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
        [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
        [  -sp,            cp*sr,             cp*cr ],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


def _rotz(theta):
    """4x4 pure rotation around z by theta radians."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0, 0],
                     [s,  c, 0, 0],
                     [0,  0, 1, 0],
                     [0,  0, 0, 1]], dtype=float)


# Precomputed fixed offsets from lerobot.urdf
_T_WORLD_BASE  = _make_T([0,       0,       0     ], [0,        0,        np.pi   ])
_T_J1_OFF      = _make_T([0,      -0.0452,  0.0165], [0,        0,        0       ])
_T_J2_OFF      = _make_T([0,      -0.0306,  0.1025], [0,       -np.pi/2,  0       ])
_T_J3_OFF      = _make_T([0.11257, -0.028,  0     ], [0,        0,        0       ])
_T_J4_OFF      = _make_T([0.0052,  -0.1349, 0     ], [0,        0,        np.pi/2 ])
_T_J5_OFF      = _make_T([-0.0601,  0,      0     ], [0,       -np.pi/2,  0       ])
_T_GRIP_CTR    = _make_T([0,        0,      0.075 ], [0,        0,        0       ])


def forward_kinematics(q):
    """Return 4×4 EE pose in the world frame for q = [q1, q2, q3, q4, q5] (rad)."""
    T = _T_WORLD_BASE
    T = T @ _T_J1_OFF @ _rotz(q[0])
    T = T @ _T_J2_OFF @ _rotz(q[1])
    T = T @ _T_J3_OFF @ _rotz(q[2])
    T = T @ _T_J4_OFF @ _rotz(q[3])
    T = T @ _T_J5_OFF @ _rotz(q[4])
    T = T @ _T_GRIP_CTR
    return T


# ── Inverse kinematics ────────────────────────────────────────────────────────

_JOINT_LIMITS = [
    (-2.0,     2.0    ),   # Shoulder_Rotation
    (-1.57,    1.57   ),   # Shoulder_Pitch
    (-1.58,    1.58   ),   # Elbow
    (-1.57,    1.57   ),   # Wrist_Pitch
    (-3.14158, 3.14158),   # Wrist_Roll
]

_W_POS = 1.0   # weight for position error (m^2)
_W_ROT = 0.3   # weight for orientation error (Frobenius)


def _ik_cost(q, target_pos, target_rot):
    T = forward_kinematics(q)
    pos_err = T[:3, 3] - target_pos
    rot_err = T[:3, :3] - target_rot
    return _W_POS * float(pos_err @ pos_err) + _W_ROT * float(np.sum(rot_err ** 2))


def inverse_kinematics(target_pos, target_rot, q0):
    """Solve IK numerically starting from q0.

    Returns (q_solution, residual_cost).
    """
    result = minimize(
        _ik_cost,
        q0,
        args=(target_pos, target_rot),
        method='L-BFGS-B',
        bounds=_JOINT_LIMITS,
        options={'maxiter': 300, 'ftol': 1e-10, 'gtol': 1e-8},
    )
    return result.x, result.fun


# ── ROS 2 node ────────────────────────────────────────────────────────────────

_HOME_Q = [np.deg2rad(0), np.deg2rad(-105),
           np.deg2rad(70), np.deg2rad(60),
           np.deg2rad(0)]

_JOINT_NAMES = ['Shoulder_Rotation', 'Shoulder_Pitch',
                'Elbow', 'Wrist_Pitch', 'Wrist_Roll']


class SetEEPosition(Node):

    def __init__(self):
        super().__init__('set_EE_position')

        # Best known joint state (warm-starts the IK solver)
        self._q = list(_HOME_Q)
        # Target EE pose: (pos np.ndarray[3], rot np.ndarray[3,3]) or None
        self._target_pose = None

        self._publisher = self.create_publisher(
            JointTrajectory, 'joint_cmds', 10)

        # Subscribe to target EE pose (geometry_msgs/Pose, world frame)
        self.create_subscription(Pose, 'target_EE_pose', self._pose_cb, 10)

        # Subscribe to joint states to warm-start the IK solver
        self.create_subscription(JointState, 'joint_states', self._js_cb, 10)

        # Solve and publish at 10 Hz
        self.create_timer(0.1, self._timer_cb)

    # ── callbacks ──────────────────────────────────────────────────────────────

    def _pose_cb(self, msg: Pose):
        pos = np.array([msg.position.x,
                        msg.position.y,
                        msg.position.z])
        rot = Rotation.from_quat([msg.orientation.x,
                                   msg.orientation.y,
                                   msg.orientation.z,
                                   msg.orientation.w]).as_matrix()
        self._target_pose = (pos, rot)

    def _js_cb(self, msg: JointState):
        for i, name in enumerate(_JOINT_NAMES):
            if name in msg.name:
                idx = msg.name.index(name)
                self._q[i] = msg.position[idx]

    def _timer_cb(self):
        if self._target_pose is None:
            return

        target_pos, target_rot = self._target_pose
        q_sol, cost = inverse_kinematics(target_pos, target_rot, self._q)

        if cost > 1e-4:
            self.get_logger().warn(
                f'IK residual high (cost={cost:.6f}); target may be unreachable')

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        point = JointTrajectoryPoint()
        point.positions = list(q_sol) + [0.0]   # append gripper (open)
        msg.points = [point]
        self._publisher.publish(msg)


# ── entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = SetEEPosition()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
