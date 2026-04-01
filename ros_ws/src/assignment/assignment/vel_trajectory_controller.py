import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.task import Future
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
from my_robot_msgs.action import ExecuteTrajectory

from .robot_kinematics import RobotKinematics


# ---------------------------------------------------------------------------
# Trajectory Generator
# ---------------------------------------------------------------------------

class ZAxisTrajectory:
    """
    Generates a trapezoidal-velocity (LTPB) oscillation trajectory along Z.
    The end-effector stays at a fixed XY position and oscillates between
    z_center - amplitude  and  z_center + amplitude with constant cruise speed,
    smoothed by parabolic acceleration/deceleration blends at each end.

    Parameters
    ----------
    x, y        : fixed horizontal position of the end-effector
    z_center    : mid-point of the vertical oscillation
    amplitude   : half-range of the Z oscillation (stroke = 2 * amplitude)
    v_max       : cruise speed during the constant-velocity phase (m/s)
    a_max       : acceleration magnitude for the blend phases (m/s²)
    n_cycles    : how many full up-down strokes to perform
    n_points    : number of interpolation samples per cycle
    """
    def __init__(self, x, y, z_center, amplitude,
                 v_max=0.05, a_max=0.1, n_cycles=2, n_points=120):
        stroke = 2.0 * amplitude          # distance for one up or down stroke

        tb = v_max / a_max                # blend (accel/decel) duration
        D  = 0.5 * a_max * tb ** 2        # distance covered during one blend

        if stroke <= 2.0 * D:
            raise ValueError(
                f"Stroke ({stroke:.3f} m) too short to reach v_max={v_max} "
                f"with a_max={a_max}. Reduce v_max, increase a_max, or increase amplitude.")

        t_stroke = 2 * tb + (stroke - 2 * D) / v_max   # duration of one half-stroke
        total_time = 2.0 * t_stroke * n_cycles
        self.time_array = np.linspace(0, total_time, n_points * n_cycles * 2)

        self.z_path = [
            self._z_at(t, z_center, amplitude, stroke, v_max, a_max, tb, D, t_stroke)
            for t in self.time_array
        ]
        n = len(self.time_array)
        self.x_path       = [float(x)] * n
        self.y_path       = [float(y)] * n
        self.rot_x_path   = [-np.pi / 2] * n
        self.rot_y_path   = [0.0] * n
        self.rot_z_path   = [0.0] * n
        self.gripper_path = [0.0] * n

    @staticmethod
    def _z_at(t, z_center, amplitude, stroke, v_max, a_max, tb, D, t_stroke):
        """
        Piecewise trapezoidal position profile.
        Each full cycle = up stroke (t_stroke) + down stroke (t_stroke).
        Within each stroke the velocity profile is: accel → cruise → decel.
        """
        cycle_time = 2.0 * t_stroke
        t_local = t % cycle_time          # time within the current cycle
        going_up = t_local < t_stroke     # first half = up, second half = down

        # Normalise to within the current stroke
        ts = t_local if going_up else (t_local - t_stroke)

        if ts < tb:                        # accel phase
            dz = 0.5 * a_max * ts ** 2
        elif ts < t_stroke - tb:           # cruise phase
            dz = D + v_max * (ts - tb)
        else:                              # decel phase
            dt = ts - (t_stroke - tb)
            dz = (stroke - D) + v_max * dt - 0.5 * a_max * dt ** 2

        z_low  = z_center - amplitude
        z_high = z_center + amplitude

        return (z_low + dz) if going_up else (z_high - dz)


# ---------------------------------------------------------------------------
# Action Server  (mirrors trajectory_controller_action_server.py)
# ---------------------------------------------------------------------------

class VelTrajectoryActionServer(Node):
    """
    ROS 2 Action Server Node.
    Receives an ExecuteTrajectory goal and tracks it using CLIK
    (Closed-Loop Inverse Kinematics) with feedforward velocity.
    """
    def __init__(self):
        super().__init__('vel_trajectory_action_server')
        self.robot = RobotKinematics()

        self._action_server = ActionServer(
            self, ExecuteTrajectory, 'execute_z_trajectory', self.execute_callback)

        self._joint_cmd_pub = self.create_publisher(JointTrajectory, 'joint_cmds', 10)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self._joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, sensor_qos)
        self._ee_pose_sub = self.create_subscription(
            PoseStamped, 'ee_pose', self.ee_pose_callback, 10)

        # Control loop parameters
        self.control_rate_hz = 100.0
        self.dt = 1.0 / self.control_rate_hz
        self.K = np.diag([0.01, 0.5, 4, 2.0, 2.0])   # CLIK feedback gain
        self.Kp = 1.5                                    # P-control gain (init phase)
        self.max_vel = 0.5
        self.tolerance = 0.05

        self.current_q_arm = None
        self.current_q_gripper = None
        self.current_ee_pose = None
        self._control_timer = None

    def joint_state_callback(self, msg):
        self.current_q_arm = np.array(msg.position[:5])
        if len(msg.position) > 5:
            self.current_q_gripper = msg.position[5]

    def ee_pose_callback(self, msg: PoseStamped):
        """Convert PoseStamped (position + quaternion) to [x, y, z, rot_x, rot_y]."""
        p = msg.pose.position
        q = msg.pose.orientation
        euler = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')
        self.current_ee_pose = np.array([p.x, p.y, p.z, euler[0], euler[1]])

    async def execute_callback(self, goal_handle):
        """
        Called when a new goal arrives.  Two-phase execution:
          Phase 1 – joint P-control moves to the first trajectory point.
          Phase 2 – CLIK tracks the full trajectory with feedforward velocity.
        Using 'async' lets the server await completion without blocking
        sensor callbacks or control timers.
        """
        self.get_logger().info('Received Z-axis trajectory goal. Preparing execution...')

        if self.current_q_arm is None:
            self.get_logger().error('Sensors not initialized. Aborting goal.')
            goal_handle.abort()
            return ExecuteTrajectory.Result()

        self._goal_handle = goal_handle

        # Unpack trajectory arrays from the goal message
        self._t_arr        = np.array(goal_handle.request.time_array)
        self._t_final      = self._t_arr[-1]
        self._x_arr        = np.array(goal_handle.request.x_path)
        self._y_arr        = np.array(goal_handle.request.y_path)
        self._z_arr        = np.array(goal_handle.request.z_path)
        self._rot_x_arr    = np.array(goal_handle.request.rot_x_path)
        self._rot_y_arr    = np.array(goal_handle.request.rot_y_path)
        self._gripper_arr  = np.array(goal_handle.request.gripper_path)

        self._tracking_errors = []
        self._tracking_times  = []
        self._trajectory_future = Future()

        # --- Compute start-position IK ---
        target_pose_0 = np.array([
            self._x_arr[0], self._y_arr[0], self._z_arr[0],
            self._rot_x_arr[0], self._rot_y_arr[0]
        ])
        # Use EE pose from /ee_pose subscriber; fall back to FK if not yet received
        if self.current_ee_pose is None:
            self.current_ee_pose = self.robot.forward_kinematics(self.current_q_arm)[:5]

        error_start = target_pose_0 - self.current_ee_pose
        error_start[3:] = (error_start[3:] + np.pi) % (2 * np.pi) - np.pi

        if np.linalg.norm(error_start) < self.tolerance:
            self.get_logger().info('Already at start position. Bypassing IK...')
            self._init_target_q_arm      = self.current_q_arm
            self._init_target_q_gripper  = self._gripper_arr[0]
        else:
            self.get_logger().info('Computing IK for start position...')
            ik_solutions = self.robot.inverse_kinematics(target_pose_0.tolist(), n_restarts=15)

            if not ik_solutions:
                self.get_logger().error(f'Start position unreachable: {target_pose_0}')
                goal_handle.abort()
                return ExecuteTrajectory.Result()

            # Pick solution closest to current configuration for smooth motion
            self._init_target_q_arm = min(
                ik_solutions,
                key=lambda q: np.linalg.norm(np.array(q) - self.current_q_arm))
            self._init_target_q_gripper = self._gripper_arr[0]

        self.get_logger().info('Moving to start position...')
        self._init_timer = self.create_timer(self.dt, self.init_control_loop)

        # Block here until the trajectory future is resolved
        await self._trajectory_future

        goal_handle.succeed()
        result = ExecuteTrajectory.Result()
        result.success = True
        result.message = 'Z-axis trajectory completed successfully.'
        return result

    def init_control_loop(self):
        """Phase 1 – joint P-control to reach the trajectory's first point."""
        error_arm = np.array(self._init_target_q_arm) - self.current_q_arm

        # Gripper may be unavailable (e.g. ID 16 serial fault) — ignore it if so
        gripper_ok = self.current_q_gripper is not None
        if gripper_ok:
            error_gripper = self._init_target_q_gripper - self.current_q_gripper
            gripper_done  = abs(error_gripper) < self.tolerance
        else:
            error_gripper = 0.0
            gripper_done  = True

        if np.linalg.norm(error_arm) < self.tolerance and gripper_done:
            self.get_logger().info('Start position reached! Launching CLIK trajectory...')
            self._init_timer.cancel()
            self.destroy_timer(self._init_timer)
            # Record the exact moment the CLIK phase begins
            self._start_time = self.get_clock().now().nanoseconds / 1e9
            self._control_timer = self.create_timer(self.dt, self.clik_control_loop)
            return

        v_cmd_arm = np.clip(self.Kp * error_arm, -self.max_vel, self.max_vel)
        v_cmd_gripper = np.clip(self.Kp * error_gripper, -self.max_vel, self.max_vel)
        self.publish_velocities(np.append(v_cmd_arm, v_cmd_gripper))

    def clik_control_loop(self):
        """Phase 2 – CLIK tracking via Jacobian inverse + feedforward velocity."""
        current_time = (self.get_clock().now().nanoseconds / 1e9) - self._start_time

        # current_ee_pose is kept fresh by ee_pose_callback; fall back to FK if stale
        if self.current_ee_pose is None:
            self.current_ee_pose = self.robot.forward_kinematics(self.current_q_arm)[:5]

        if current_time >= self._t_final:
            self.get_logger().info('Z-axis trajectory complete. Stopping.')
            self._control_timer.cancel()
            self.publish_velocities(np.zeros(6))

            if self._tracking_errors:
                errors = np.array(self._tracking_errors)
                self.get_logger().info(
                    f'Pos RMSE: {np.sqrt(np.mean(errors[:, 0]**2)):.5f} m  |  '
                    f'Max: {np.max(errors[:, 0]):.5f} m')

            self._trajectory_future.set_result(True)
            return

        # Interpolate desired pose and feedforward velocity at current_time
        desired_pose = np.array([
            np.interp(current_time, self._t_arr, self._x_arr),
            np.interp(current_time, self._t_arr, self._y_arr),
            np.interp(current_time, self._t_arr, self._z_arr),
            np.interp(current_time, self._t_arr, self._rot_x_arr),
            np.interp(current_time, self._t_arr, self._rot_y_arr),
        ])
        target_gripper = np.interp(current_time, self._t_arr, self._gripper_arr)

        actual_pose = self.current_ee_pose
        error = desired_pose - actual_pose
        error[3:] = (error[3:] + np.pi) % (2 * np.pi) - np.pi  # wrap angle error

        self._tracking_errors.append((np.linalg.norm(error[:3]), np.linalg.norm(error[3:])))
        self._tracking_times.append(current_time)

        # Feedforward via finite difference of desired trajectory
        prev_pose = np.array([
            np.interp(current_time - self.dt, self._t_arr, self._x_arr),
            np.interp(current_time - self.dt, self._t_arr, self._y_arr),
            np.interp(current_time - self.dt, self._t_arr, self._z_arr),
            np.interp(current_time - self.dt, self._t_arr, self._rot_x_arr),
            np.interp(current_time - self.dt, self._t_arr, self._rot_y_arr),
        ])
        v_ff = (desired_pose - prev_pose) / self.dt

        # 5-DOF inverse Jacobian control
        J_inv, _ = self.robot.jacobian_inverse(self.current_q_arm)
        q_dot_arm = J_inv @ (v_ff + self.K @ error)

        # Enforce joint limits
        for i in range(5):
            low, high = self.robot.joint_bounds[i]
            if self.current_q_arm[i] >= high and q_dot_arm[i] > 0:
                q_dot_arm[i] = 0.0
            elif self.current_q_arm[i] <= low and q_dot_arm[i] < 0:
                q_dot_arm[i] = 0.0

        # Gripper P-control (skip if gripper state unavailable due to hardware fault)
        if self.current_q_gripper is not None:
            v_gripper = np.clip(
                self.Kp * (target_gripper - self.current_q_gripper), -self.max_vel, self.max_vel)
        else:
            v_gripper = 0.0

        self.publish_velocities(np.append(q_dot_arm, v_gripper))

        feedback_msg = ExecuteTrajectory.Feedback()
        feedback_msg.current_time = current_time
        feedback_msg.percent_complete = (current_time / self._t_final) * 100.0
        self._goal_handle.publish_feedback(feedback_msg)

    def publish_velocities(self, velocities):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.robot.joint_names + ['Gripper']
        point = JointTrajectoryPoint()
        point.velocities = list(velocities)
        point.time_from_start = rclpy.time.Duration(seconds=0.0).to_msg()
        traj_msg.points.append(point)
        self._joint_cmd_pub.publish(traj_msg)


# ---------------------------------------------------------------------------
# Action Client  (mirrors LTPB_trajectory_action_client.py)
# ---------------------------------------------------------------------------

class VelTrajectoryActionClient(Node):
    """
    ROS 2 Action Client Node.
    Generates a Z-axis oscillation trajectory and sends it to the server.
    """
    def __init__(self):
        super().__init__('vel_trajectory_action_client')
        self._action_client = ActionClient(self, ExecuteTrajectory, 'execute_z_trajectory')
        self.action_result   = None
        self.is_action_done  = False
        self.sensors_ready   = False

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(JointState, 'joint_states', self._js_cb, sensor_qos)

    def _js_cb(self, msg):
        if not self.sensors_ready and len(msg.position) >= 5:
            self.sensors_ready = True
            self.get_logger().info('Joint states received — hardware ready.')

    def send_z_trajectory(self, x, y, z_center, amplitude,
                          v_max=0.05, a_max=0.1, n_cycles=2):
        """Generate a Z-axis oscillation trajectory and send it as a goal."""
        self.is_action_done = False
        self.action_result  = None

        self.get_logger().info(
            f'Generating Z trajectory: x={x}, y={y}, '
            f'z_center={z_center:.3f}, amplitude={amplitude:.3f}, '
            f'v_max={v_max} m/s, a_max={a_max} m/s², cycles={n_cycles}')

        traj = ZAxisTrajectory(x, y, z_center, amplitude, v_max, a_max, n_cycles)

        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.time_array   = traj.time_array.tolist()
        goal_msg.x_path       = traj.x_path
        goal_msg.y_path       = traj.y_path
        goal_msg.z_path       = traj.z_path
        goal_msg.rot_x_path   = traj.rot_x_path
        goal_msg.rot_y_path   = traj.rot_y_path
        goal_msg.rot_z_path   = traj.rot_z_path
        goal_msg.gripper_path = traj.gripper_path

        self._action_client.wait_for_server()
        self.get_logger().info('Server found! Sending Z-axis trajectory goal...')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'Progress: {fb.percent_complete:.1f}%', throttle_duration_sec=2.0)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by server.')
            self.is_action_done = True
            return
        self.get_logger().info('Goal accepted! Executing Z-axis trajectory...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.action_result  = future.result().result
        self.is_action_done = True
        self.get_logger().info(f'Result: {self.action_result.message}')


# ---------------------------------------------------------------------------
# Entry Points
# ---------------------------------------------------------------------------

def main_server(args=None):
    rclpy.init(args=args)
    node = VelTrajectoryActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main_client(args=None):
    rclpy.init(args=args)
    node = VelTrajectoryActionClient()

    # Block until the hardware has published at least one valid joint state
    node.get_logger().info('Waiting for hardware sensors to become ready...')
    while rclpy.ok() and not node.sensors_ready:
        rclpy.spin_once(node, timeout_sec=0.1)

    # Oscillate 10 cm around a 20 cm height at (0.0, 0.3)
    node.send_z_trajectory(x=0.0, y=0.3, z_center=0.2, amplitude=0.1,
                           v_max=0.05, a_max=0.1, n_cycles=5)

    while rclpy.ok() and not node.is_action_done:
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()


# Default entry point (used by the launch file) runs the server
def main(args=None):
    main_server(args)


if __name__ == '__main__':
    main()
