"""
Pick-and-place node with Cartesian-space proportional velocity control.

Implements a position-based impedance-like controller for bidirectional
pick-and-place (A→B, then B→A) on the SO-ARM100.  The robot is
velocity-controlled so true torque-based impedance is not available.
Instead we use directional stiffness gains (k_z << k_xy) to produce a
soft Z-axis approach that behaves like impedance without a force sensor.

Control law (Cartesian proportional + damping):
    e_pos  = x_des - x_cur                     # (6,) Cartesian error
    e_vel  = -dx_cur                            # (6,) velocity damping
    v_cart = K_m @ e_pos + D_m @ e_vel         # (6,) Cartesian velocity
    v_cart = clip(v_cart, ±max_cart_vel)
    dq     = J_inv @ v_cart                    # (5,) joint velocity
    dq     = clip(dq, ±max_joint_vel)

Phase 2 hybrid upgrade path is documented in ImpedanceController.compute().

Usage:
    ros2 run assignment pick_place_impedance
"""

import json
import os
import subprocess
import sys
import time
from enum import Enum, auto
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# ---------------------------------------------------------------------------
# Project-root discovery — verbatim from vel_trajectory_follower.py
# ---------------------------------------------------------------------------

def _find_project_root(start: Path) -> Path:
    """Walk up from start until finding the directory containing ros_ws/ and assignment/."""
    for candidate in [start, *start.parents]:
        if (candidate / 'ros_ws').is_dir() and (candidate / 'assignment').is_dir():
            return candidate
    raise RuntimeError(f'Cannot locate project root (edubot/) from {start}.')


_PROJECT_ROOT = _find_project_root(Path(__file__).resolve())
_KINEMATICS_DIR = str(_PROJECT_ROOT / 'assignment')

if _KINEMATICS_DIR not in sys.path:
    sys.path.insert(0, _KINEMATICS_DIR)

from robot_kinematics import RobotKinematics  # noqa: E402

DEFAULT_CONFIG_FILE = os.path.join(
    get_package_share_directory('assignment'), 'config', 'pick_place_config.json'
)

JOINT_NAMES = [
    'Shoulder_Rotation',
    'Shoulder_Pitch',
    'Elbow',
    'Wrist_Pitch',
    'Wrist_Roll',
    'Gripper',
]


# ---------------------------------------------------------------------------
# State machine enum
# ---------------------------------------------------------------------------

class State(Enum):
    HOME = auto()
    APPROACH_OBJECT = auto()
    DESCEND_TO_OBJECT = auto()
    GRASP = auto()
    LIFT = auto()
    MOVE_TO_TARGET = auto()
    DESCEND_TO_PLACE = auto()
    RELEASE = auto()
    LIFT_AFTER_PLACE = auto()
    RETURN_HOME = auto()
    SWAP_LOCATIONS = auto()
    DONE = auto()


# ---------------------------------------------------------------------------
# ImpedanceController — Cartesian proportional velocity with directional gains
# ---------------------------------------------------------------------------

class ImpedanceController:
    """
    Cartesian-space proportional velocity controller with directional stiffness.

    K_z << K_xy produces a soft Z-axis approach (compliant descent), which
    mirrors impedance behaviour without requiring torque control or a force sensor.

    Phase 2 hybrid upgrade: replace Phase-1 position control on constrained axes
    with force control.  The contact_mask parameter (always zeros in Phase 1)
    marks axes under contact.  To upgrade, subscribe to a force/torque topic and
    change compute() as documented below.
    """

    def __init__(
        self,
        k_xy: float,
        k_z: float,
        k_rot: float,
        d_xy: float,
        d_z: float,
        d_rot: float,
        max_cart_vel: float,
    ):
        self._K = self._build_stiffness_matrix(k_xy, k_z, k_rot)
        self._D = self._build_damping_matrix(d_xy, d_z, d_rot)
        self._max_cart_vel = max_cart_vel

    @staticmethod
    def _build_stiffness_matrix(k_xy, k_z, k_rot) -> np.ndarray:
        """Build 6×6 diagonal stiffness matrix [vx, vy, vz, vpitch, vroll, vyaw]."""
        return np.diag([k_xy, k_xy, k_z, k_rot, k_rot, k_rot])

    @staticmethod
    def _build_damping_matrix(d_xy, d_z, d_rot) -> np.ndarray:
        """Build 6×6 diagonal damping matrix."""
        return np.diag([d_xy, d_xy, d_z, d_rot, d_rot, d_rot])

    def compute(
        self,
        x_des: np.ndarray,
        x_cur: np.ndarray,
        dx_cur: np.ndarray,
        J_inv: np.ndarray,
        max_joint_vel: float,
        contact_mask: np.ndarray = None,
    ) -> np.ndarray:
        """
        Compute joint velocity command from desired/current Cartesian poses.

        Phase 1 (current): pure position control on all axes.
        Phase 2 hybrid upgrade (single change here):
            For constrained axes where contact_mask[i] == True:
                v_cart[i] = K_f[i,i] * (f_des[i] - f_cur[i])
                # subscribe to a force/torque topic for f_cur
            For free axes where contact_mask[i] == False:
                v_cart[i] unchanged (position control as below)
            contact_mask[2] (Z-axis) is the primary candidate during DESCEND states.

        Args:
            x_des:       (6,) desired EE pose [x, y, z, pitch, roll, yaw]
            x_cur:       (6,) current EE pose from FK
            dx_cur:      (6,) current EE velocity (finite-diff, LP-filtered)
            J_inv:       (5,6) Jacobian pseudo-inverse from RobotKinematics
            max_joint_vel: per-joint velocity safety clamp (rad/s)
            contact_mask: (6,) bool array, True on constrained axes (Phase 1: all False)

        Returns:
            dq: (5,) joint velocity command, clipped to ±max_joint_vel
        """
        # Positional error in Cartesian space
        e_pos = x_des - x_cur          # (6,)

        # Wrap orientation error components to [-π, π] to handle angle discontinuities
        # (e.g. rot_x near ±π can jump by 2π without wrapping)
        e_pos[3:] = (e_pos[3:] + np.pi) % (2 * np.pi) - np.pi

        # Damping term: oppose current velocity (desired velocity = 0 → hold)
        e_vel = -dx_cur                 # (6,)

        # Cartesian velocity command from stiffness + damping
        v_cart = self._K @ e_pos + self._D @ e_vel     # (6,)

        # Safety clamp on Cartesian velocity
        v_cart = np.clip(v_cart, -self._max_cart_vel, self._max_cart_vel)

        # Map to joint velocities via Jacobian pseudo-inverse
        dq = J_inv @ v_cart             # (5,)

        # Per-joint safety clamp
        dq = np.clip(dq, -max_joint_vel, max_joint_vel)

        return dq


# ---------------------------------------------------------------------------
# GripperController — bang-bang position servo
# ---------------------------------------------------------------------------

class GripperController:
    """
    Bang-bang gripper position servo.

    Applies ±vel_rad_s until the gripper is within dead_band_rad of the target.
    The velocity is published as the 6th element of the JointTrajectory velocities
    list (confirmed in robot.cpp:115-127 and robot_sim.cpp:16-35).
    """

    def __init__(
        self,
        open_angle: float,
        close_angle: float,
        vel_rad_s: float,
        dead_band: float,
    ):
        self._open_angle = open_angle
        self._close_angle = close_angle
        self._vel = vel_rad_s
        self._dead_band = dead_band

    def command_open(self, current_angle: float) -> float:
        """Return positive velocity to open, or 0.0 when settled."""
        error = self._open_angle - current_angle
        return self._vel if abs(error) > self._dead_band else 0.0

    def command_close(self, current_angle: float) -> float:
        """Return negative velocity to close, or 0.0 when settled."""
        error = self._close_angle - current_angle
        return -self._vel if abs(error) > self._dead_band else 0.0

    def command_hold(self, current_angle: float) -> float:
        """Hold current gripper position."""
        return 0.0

    def is_open(self, current_angle: float) -> bool:
        """True when the gripper is within dead_band of the open angle."""
        return abs(current_angle - self._open_angle) < self._dead_band

    def is_closed(self, current_angle: float) -> bool:
        """True when the gripper is within dead_band of the close angle."""
        return abs(current_angle - self._close_angle) < self._dead_band


# ---------------------------------------------------------------------------
# PickPlaceImpedanceNode
# ---------------------------------------------------------------------------

class PickPlaceImpedanceNode(Node):
    """
    ROS 2 node for bidirectional pick-and-place with impedance-like velocity control.

    Cycle 1: pick from A, place at B.
    Cycle 2: pick from B, place at A.
    Stops after max_cycles (configurable in pick_place_config.json).
    """

    def __init__(self):
        super().__init__('pick_place_impedance')

        # Ensure the velocity-mode simulator is running
        self._sim_proc = None
        self._ensure_velocity_sim()

        # Load kinematics (triggers SymPy compilation — takes a few seconds)
        self.get_logger().info('Compiling robot kinematics (SymPy lambdify)...')
        self._robot = RobotKinematics()
        self.get_logger().info('RobotKinematics ready.')

        # Load task config
        self._cfg = self._load_config(DEFAULT_CONFIG_FILE)

        # Unpack impedance parameters
        imp = self._cfg['impedance']
        self._controller = ImpedanceController(
            k_xy=imp['k_xy'],
            k_z=imp['k_z'],
            k_rot=imp['k_rot'],
            d_xy=imp['d_xy'],
            d_z=imp['d_z'],
            d_rot=imp['d_rot'],
            max_cart_vel=imp['max_cart_vel_m_s'],
        )
        self._max_joint_vel = imp['max_joint_vel_rad_s']

        # Unpack gripper parameters
        grp = self._cfg['gripper']
        self._gripper = GripperController(
            open_angle=grp['open_angle_rad'],
            close_angle=grp['close_angle_rad'],
            vel_rad_s=grp['vel_rad_s'],
            dead_band=grp['dead_band_rad'],
        )

        # Unpack motion thresholds
        mot = self._cfg['motion']
        self._thresh_approach = mot['threshold_approach_m']
        self._thresh_pick = mot['threshold_pick_m']
        self._thresh_lift = mot['threshold_lift_m']
        self._thresh_home = mot['threshold_home_m']
        self._dwell_grasp = mot['dwell_grasp_s']
        self._dwell_release = mot['dwell_release_s']
        self._descent_timeout = mot['descent_timeout_s']
        self._max_cycles = mot['max_cycles']
        self._approach_z = mot['approach_z']

        # Pick/place location pair (swapped each cycle)
        locs = self._cfg['locations']
        self._loc_pick = np.array([locs['A']['x'], locs['A']['y'], locs['A']['z']])
        self._loc_place = np.array([locs['B']['x'], locs['B']['y'], locs['B']['z']])

        # Home joint configuration
        self._home_q = np.array(self._cfg['home']['joint_angles_rad'])

        # Fixed EE orientation: rot_x=π (gripper down), rot_y=0, rot_z from home FK
        _home_pose = np.array(self._robot.forward_kinematics(self._home_q)).flatten()
        self._fixed_orientation = np.array([np.pi, 0.0, _home_pose[5]])

        # Runtime state
        self._current_q = None       # (6,) — 5 arm joints + 1 gripper, from joint_states
        self._state = State.HOME
        self._cycle = 0
        self._state_entry_time = self.get_clock().now()
        self._dwell_start = None     # used for timed dwell states (GRASP, RELEASE)

        # EE velocity estimation (finite-difference + low-pass filter)
        self._x_prev = None          # previous EE pose for finite-diff
        self._dx_cur = np.zeros(6)   # low-pass filtered EE velocity
        self._lp_alpha = 0.3         # LP filter coefficient

        # State handler dispatch table
        self._handlers = {
            State.HOME:               self._state_home,
            State.APPROACH_OBJECT:    self._state_approach_object,
            State.DESCEND_TO_OBJECT:  self._state_descend_to_object,
            State.GRASP:              self._state_grasp,
            State.LIFT:               self._state_lift,
            State.MOVE_TO_TARGET:     self._state_move_to_target,
            State.DESCEND_TO_PLACE:   self._state_descend_to_place,
            State.RELEASE:            self._state_release,
            State.LIFT_AFTER_PLACE:   self._state_lift_after_place,
            State.RETURN_HOME:        self._state_return_home,
            State.SWAP_LOCATIONS:     self._state_swap_locations,
            State.DONE:               self._state_done,
        }

        # ROS interfaces
        self._pub = self.create_publisher(JointTrajectory, 'joint_cmds', 10)
        self._sub = self.create_subscription(
            JointState, 'joint_states', self._on_joint_states, qos_profile_sensor_data
        )
        self.create_timer(1.0 / 25.0, self._control_loop)   # 25 Hz

        self.get_logger().info(
            f'PickPlaceImpedance ready. '
            f'A={self._loc_pick}, B={self._loc_place}, max_cycles={self._max_cycles}'
        )

    # ------------------------------------------------------------------
    # Setup helpers
    # ------------------------------------------------------------------

    def _find_project_root(self, start: Path) -> Path:
        """Walk up from start until finding the directory containing ros_ws/ and assignment/."""
        return _find_project_root(start)

    def _load_config(self, path: str) -> dict:
        """Load and return the JSON configuration file."""
        self.get_logger().info(f'Loading config from: {path}')
        with open(path, 'r') as f:
            return json.load(f)

    def _ensure_velocity_sim(self) -> None:
        """Launch lerobot_sim in velocity mode if it is not already running."""
        try:
            result = subprocess.run(
                ['ros2', 'node', 'list'], capture_output=True, text=True, timeout=5
            )
            running_nodes = result.stdout
        except (subprocess.TimeoutExpired, FileNotFoundError):
            running_nodes = ''

        if 'lerobot_sim' in running_nodes:
            self.get_logger().info('[OK] lerobot_sim already running.')
            return

        self.get_logger().info(
            '[INFO] lerobot_sim not found — launching in velocity mode (no RViz)...'
        )
        self._sim_proc = subprocess.Popen(
            ['ros2', 'launch', 'lerobot', 'sim_velocity.launch.py', 'use_rviz:=false'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )

        for _ in range(20):
            time.sleep(0.5)
            try:
                r = subprocess.run(
                    ['ros2', 'node', 'list'], capture_output=True, text=True, timeout=5
                )
                if 'lerobot_sim' in r.stdout:
                    self.get_logger().info('[OK] lerobot_sim is running in velocity mode.')
                    return
            except (subprocess.TimeoutExpired, FileNotFoundError):
                pass

        self.get_logger().error(
            '[ERROR] lerobot_sim did not start within 10 s. '
            'Run manually: ros2 launch lerobot sim_velocity.launch.py'
        )

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def _on_joint_states(self, msg: JointState) -> None:
        """Store current joint positions (arm + gripper) from /joint_states.

        Arm joints are clamped to their kinematic bounds — the simulator can
        report positions slightly outside the modelled limits during fast motion,
        which would cause FK / Jacobian calls to raise ValueError.
        """
        name_to_pos = dict(zip(msg.name, msg.position))
        q_raw = np.array([name_to_pos.get(n, 0.0) for n in JOINT_NAMES])

        # Clamp arm joints [0:5] to their allowed ranges; leave gripper [5] unclamped
        bounds = self._robot.joint_bounds
        arm_q_clamped = np.array([
            np.clip(q_raw[i], bounds[i][0], bounds[i][1]) for i in range(5)
        ])
        self._current_q = np.concatenate([arm_q_clamped, [q_raw[5]]])

    def _control_loop(self) -> None:
        """Main 25 Hz control step — dispatch to current state handler."""
        if self._current_q is None:
            self.get_logger().warn('Waiting for joint states...', throttle_duration_sec=1.0)
            return

        # Dispatch to the active state handler
        self._handlers[self._state]()

    # ------------------------------------------------------------------
    # State handlers
    # ------------------------------------------------------------------

    def _state_home(self) -> None:
        """Drive all arm joints to the home configuration."""
        # Compute FK at home angles to get the home EE pose
        home_pose = np.array(
            self._robot.forward_kinematics(self._home_q)
        ).flatten()

        err = self._run_impedance_step(home_pose, self._gripper.command_open)
        if err < self._thresh_home:
            self.get_logger().info(f'HOME reached (err={err*100:.1f} cm). Cycle {self._cycle+1}.')
            self._transition_to(State.APPROACH_OBJECT)

    def _state_approach_object(self) -> None:
        """Move EE to approach height above the pick location."""
        target = np.array([
            self._loc_pick[0], self._loc_pick[1], self._approach_z
        ])
        target_pose = self._make_translation_pose(target)

        err = self._run_impedance_step(target_pose, self._gripper.command_open)
        if err < self._thresh_approach:
            self.get_logger().info('APPROACH_OBJECT reached.')
            self._transition_to(State.DESCEND_TO_OBJECT)

    def _state_descend_to_object(self) -> None:
        """Lower EE to the pick location. Soft Z gain provides gentle descent."""
        target_pose = self._make_translation_pose(self._loc_pick)

        err = self._run_impedance_step(target_pose, self._gripper.command_open)
        elapsed = (self.get_clock().now() - self._state_entry_time).nanoseconds * 1e-9

        if err < self._thresh_pick:
            self.get_logger().info(f'DESCEND_TO_OBJECT reached (err={err*100:.1f} cm).')
            self._transition_to(State.GRASP)
        elif elapsed > self._descent_timeout:
            self.get_logger().warn(
                f'Descent timeout ({self._descent_timeout:.1f} s) — proceeding to GRASP.'
            )
            self._transition_to(State.GRASP)

    def _state_grasp(self) -> None:
        """Close the gripper and dwell to secure the object."""
        arm_q = self._current_q[:5]
        x_cur = np.array(self._robot.forward_kinematics(arm_q)).flatten()
        # Hold current EE pose while closing
        self._run_impedance_step(x_cur, lambda _: 0.0, hold_arm=True)

        gripper_vel = self._gripper.command_close(self._current_q[5])
        self._publish_velocity(np.zeros(5), gripper_vel)

        # Start dwell timer once the gripper is physically closed
        if self._gripper.is_closed(self._current_q[5]):
            if self._dwell_start is None:
                self._dwell_start = self.get_clock().now()
                self.get_logger().info('Gripper closed — dwelling.')
            elapsed = (self.get_clock().now() - self._dwell_start).nanoseconds * 1e-9
            if elapsed >= self._dwell_grasp:
                self._dwell_start = None
                self._transition_to(State.LIFT)

    def _state_lift(self) -> None:
        """Lift the object to approach height."""
        target = np.array([
            self._loc_pick[0], self._loc_pick[1], self._approach_z
        ])
        target_pose = self._make_translation_pose(target)

        err = self._run_impedance_step(target_pose, self._gripper.command_hold)
        if err < self._thresh_lift:
            self.get_logger().info('LIFT reached.')
            self._transition_to(State.MOVE_TO_TARGET)

    def _state_move_to_target(self) -> None:
        """Move EE to approach height above the place location."""
        target = np.array([
            self._loc_place[0], self._loc_place[1], self._approach_z
        ])
        target_pose = self._make_translation_pose(target)

        err = self._run_impedance_step(target_pose, self._gripper.command_hold)
        if err < self._thresh_approach:
            self.get_logger().info('MOVE_TO_TARGET reached.')
            self._transition_to(State.DESCEND_TO_PLACE)

    def _state_descend_to_place(self) -> None:
        """Lower EE to the place location."""
        target_pose = self._make_translation_pose(self._loc_place)

        err = self._run_impedance_step(target_pose, self._gripper.command_hold)
        elapsed = (self.get_clock().now() - self._state_entry_time).nanoseconds * 1e-9

        if err < self._thresh_pick:
            self.get_logger().info('DESCEND_TO_PLACE reached.')
            self._transition_to(State.RELEASE)
        elif elapsed > self._descent_timeout:
            self.get_logger().warn('Descent-to-place timeout — proceeding to RELEASE.')
            self._transition_to(State.RELEASE)

    def _state_release(self) -> None:
        """Open the gripper and dwell before lifting."""
        # Hold arm in place while opening
        arm_q = self._current_q[:5]
        x_cur = np.array(self._robot.forward_kinematics(arm_q)).flatten()
        self._run_impedance_step(x_cur, lambda _: 0.0, hold_arm=True)

        gripper_vel = self._gripper.command_open(self._current_q[5])
        self._publish_velocity(np.zeros(5), gripper_vel)

        if self._gripper.is_open(self._current_q[5]):
            if self._dwell_start is None:
                self._dwell_start = self.get_clock().now()
                self.get_logger().info('Gripper open — dwelling.')
            elapsed = (self.get_clock().now() - self._dwell_start).nanoseconds * 1e-9
            if elapsed >= self._dwell_release:
                self._dwell_start = None
                self._transition_to(State.LIFT_AFTER_PLACE)

    def _state_lift_after_place(self) -> None:
        """Lift to approach height after placing the object."""
        target = np.array([
            self._loc_place[0], self._loc_place[1], self._approach_z
        ])
        target_pose = self._make_translation_pose(target)

        err = self._run_impedance_step(target_pose, self._gripper.command_open)
        if err < self._thresh_lift:
            self.get_logger().info('LIFT_AFTER_PLACE reached.')
            self._transition_to(State.RETURN_HOME)

    def _state_return_home(self) -> None:
        """Return to home configuration before next cycle or finishing."""
        home_pose = np.array(
            self._robot.forward_kinematics(self._home_q)
        ).flatten()

        err = self._run_impedance_step(home_pose, self._gripper.command_open)
        if err < self._thresh_home:
            self.get_logger().info('RETURN_HOME reached.')
            self._transition_to(State.SWAP_LOCATIONS)

    def _state_swap_locations(self) -> None:
        """Swap pick/place locations and advance cycle counter.  No motion required."""
        self._cycle += 1
        self.get_logger().info(
            f'Completed cycle {self._cycle}. Swapping A↔B locations.'
        )
        # Swap pick and place targets for the next cycle
        self._loc_pick, self._loc_place = self._loc_place, self._loc_pick

        if self._cycle >= self._max_cycles:
            self._transition_to(State.DONE)
        else:
            self._transition_to(State.HOME)

    def _state_done(self) -> None:
        """Terminal state — publish zero velocities."""
        self._publish_stop()
        self.get_logger().info(
            f'Pick-and-place complete ({self._cycle} cycle(s)). Node idle.',
            throttle_duration_sec=5.0,
        )

    # ------------------------------------------------------------------
    # Core helpers
    # ------------------------------------------------------------------

    def _make_translation_pose(self, xyz: np.ndarray) -> np.ndarray:
        """
        Build a 6-DOF target pose that commands only XYZ translation.

        Orientation components are set to the current EE orientation so the
        controller does not fight the 5-DOF kinematic constraint.
        This is used for APPROACH, DESCEND, LIFT, and MOVE states.
        """
        arm_q = self._current_q[:5]
        x_cur = np.array(self._robot.forward_kinematics(arm_q)).flatten()
        target = np.copy(x_cur)
        target[:3] = xyz                       # commanded XYZ
        target[3:] = self._fixed_orientation   # enforce rot_x=π, rot_y=0, rot_z=const
        return target

    def _run_impedance_step(
        self,
        target_pose: np.ndarray,
        gripper_cmd_fn,
        hold_arm: bool = False,
    ) -> float:
        """
        Execute one impedance control step and publish joint velocities.

        Args:
            target_pose:    (6,) desired EE pose
            gripper_cmd_fn: callable(current_gripper_angle) → float velocity
            hold_arm:       if True, publish zero arm velocities (gripper-only step)

        Returns:
            Position error norm (XYZ only) in metres.
        """
        arm_q = self._current_q[:5]
        x_cur = np.array(self._robot.forward_kinematics(arm_q)).flatten()

        # Update finite-difference EE velocity estimate
        dx = self._update_dx_cur(x_cur)

        if hold_arm:
            dq = np.zeros(5)
        else:
            J_inv, is_singular = self._robot.jacobian_inverse(arm_q)
            if is_singular:
                self.get_logger().warn(
                    'Near singularity — DLS damping active.', throttle_duration_sec=0.5
                )
            dq = self._controller.compute(
                x_des=target_pose,
                x_cur=x_cur,
                dx_cur=dx,
                J_inv=J_inv,
                max_joint_vel=self._max_joint_vel,
            )

        gripper_vel = gripper_cmd_fn(self._current_q[5]) if gripper_cmd_fn else 0.0
        self._publish_velocity(dq, gripper_vel)

        return self._position_error(target_pose[:3])

    def _position_error(self, target_xyz: np.ndarray) -> float:
        """Euclidean XYZ distance between current EE and target."""
        arm_q = self._current_q[:5]
        x_cur = np.array(self._robot.forward_kinematics(arm_q)).flatten()
        return float(np.linalg.norm(target_xyz - x_cur[:3]))

    def _update_dx_cur(self, x_cur: np.ndarray) -> np.ndarray:
        """
        Estimate EE velocity via finite differences with LP filter.

        Phase 1: returns zeros (no damping contribution) for the first tick
        and until the filter warms up.  Can be enabled fully by reducing
        the Phase-1 guard below.
        """
        dt = 1.0 / 25.0   # 25 Hz control loop

        if self._x_prev is None:
            # Phase 1: no estimate on first tick
            self._x_prev = x_cur.copy()
            return np.zeros(6)

        # Finite-difference velocity
        dx_raw = (x_cur - self._x_prev) / dt

        # Low-pass filter: α=0.3 → smooth but responsive
        self._dx_cur = self._lp_alpha * dx_raw + (1.0 - self._lp_alpha) * self._dx_cur
        self._x_prev = x_cur.copy()

        return self._dx_cur

    def _apply_joint_limit_clamp(self, q_dot: np.ndarray) -> np.ndarray:
        """
        Zero any velocity component that would drive a joint further past its limit.

        This is the actual enforcement of joint limits — the clamping in
        _on_joint_states only prevents FK from crashing on out-of-range read-backs;
        it does NOT stop the Jacobian controller from commanding motion that violates
        the limits.  Here we apply the standard velocity-level limit guard:

            if q[i] >= upper[i] and dq[i] > 0 → dq[i] = 0  (would go higher)
            if q[i] <= lower[i] and dq[i] < 0 → dq[i] = 0  (would go lower)
        """
        q_dot_safe = q_dot.copy()
        bounds = self._robot.joint_bounds
        for i in range(5):
            lo, hi = bounds[i]
            if self._current_q[i] >= hi and q_dot_safe[i] > 0.0:
                q_dot_safe[i] = 0.0
            elif self._current_q[i] <= lo and q_dot_safe[i] < 0.0:
                q_dot_safe[i] = 0.0
        return q_dot_safe

    def _publish_velocity(self, q_dot: np.ndarray, gripper_vel: float = 0.0) -> None:
        """Publish joint velocity command to /joint_cmds (5 arm joints + gripper)."""
        q_dot_safe = self._apply_joint_limit_clamp(q_dot)
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.velocities = [float(v) for v in q_dot_safe] + [float(gripper_vel)]
        msg.points = [point]
        self._pub.publish(msg)

    def _publish_stop(self) -> None:
        """Publish zero velocity command to stop all joints."""
        self._publish_velocity(np.zeros(5), 0.0)

    def _transition_to(self, new_state: State) -> None:
        """Switch to a new state and reset the state-entry timestamp."""
        self.get_logger().info(
            f'State: {self._state.name} → {new_state.name}'
        )
        self._state = new_state
        self._state_entry_time = self.get_clock().now()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceImpedanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_stop()
        node.destroy_node()
        if node._sim_proc is not None:
            node._sim_proc.terminate()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
