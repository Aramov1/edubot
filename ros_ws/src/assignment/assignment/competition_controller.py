import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.task import Future
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from my_robot_msgs.action import ExecuteTrajectory
import time

from .robot_kinematics import RobotKinematics


# ---------------------------------------------------------------------------
# Cartesian positions [x, y, z] in metres  — update for your hardware setup
# ---------------------------------------------------------------------------

PICK   = [0.04, 0.16, 0.04]   # Pick position
PLACE1 = [0.10, 0.12, 0.04]   # Place position 1
PLACE2 = [0.10, 0.12, 0.06]   # Place position 2
PLACE3 = [0.10, 0.12, 0.08]   # Place position 3

LIFT_HEIGHT = 0.05  # metres — arc height used for all LTPB moves

GRIPPER_OPEN   = 0.5
GRIPPER_CLOSED = 0.2


# ---------------------------------------------------------------------------
# Trajectory Generator
# ---------------------------------------------------------------------------

class LTPBTrajectory:
    """
    Linear Trajectory with Parabolic Blends (LTPB).

    Motion profile (4 phases):
      1. Dwell at P1      — gripper opens/closes over t_grip seconds
      2. Travel P1→P4     — lift up, translate laterally, lower down
      3. Dwell at P4      — gripper opens fully over t_grip seconds
      4. Retract from P4  — lift straight up by lift_height so the arm clears
                            stacked blocks before the next move
    """
    def __init__(self, P1, P4, lift_height, v_max=0.1, a_max=0.2, grasp=True):
        self.P1 = np.array(P1)
        self.P4 = np.array(P4)
        self.lift_height = lift_height
        self.v_max = v_max
        self.a_max = a_max

        self.dx = self.P4[0] - self.P1[0]
        self.dy = self.P4[1] - self.P1[1]
        self.L = np.sqrt(self.dx**2 + self.dy**2)
        self.H = self.lift_height

        self.theta = 0.0 if self.L == 0 else np.arctan2(self.dy, self.dx)
        self.cos_theta = np.cos(self.theta)
        self.sin_theta = np.sin(self.theta)

        self._compute_timing_parameters()

        # t_grip: dwell time at each endpoint — long enough for the gripper to
        # fully actuate (0.2 rad gap / 0.5 rad/s max_vel ≈ 0.4 s; 1.0 s is safe)
        self.t_grip = 1.0
        # Retract duration: same trapezoidal profile as the lift-up phase (0→H)
        self.t_retract = self.t3

        t_end_travel = self.t_grip + self.t7
        t_end_dwell  = t_end_travel + self.t_grip
        total_time   = t_end_dwell + self.t_retract

        self.time_array = np.linspace(0, total_time, 200)

        self.x_path = []
        self.y_path = []
        self.z_path = []
        self.gripper_path = []

        for t in self.time_array:
            if t < self.t_grip:
                # Phase 1: dwell at P1, actuate gripper
                x = self.P1[0]
                y = self.P1[1]
                z = self.P1[2]
                grip = GRIPPER_CLOSED if grasp else GRIPPER_OPEN

            elif t < t_end_travel:
                # Phase 2: travel from P1 to P4 along LTPB arc.
                # _local_y is symmetric and returns to 0 at t7, so without
                # correction the arc always ends at P1[2]. A linear Z
                # interpolation (proportional to horizontal progress) shifts
                # the arc so it lands exactly at P4[2], handling stacking.
                ltpb_t    = t - self.t_grip
                x_prog    = self._local_x(ltpb_t) / self.L if self.L > 0 else 0.0
                x = self.P1[0] + self._local_x(ltpb_t) * self.cos_theta
                y = self.P1[1] + self._local_x(ltpb_t) * self.sin_theta
                z = self.P1[2] + self._local_y(ltpb_t) + (self.P4[2] - self.P1[2]) * x_prog
                grip = GRIPPER_CLOSED if grasp else GRIPPER_OPEN

            elif t < t_end_dwell:
                # Phase 3: dwell at P4, open gripper
                # The full t_grip dwell ensures the gripper is open before
                # any arm motion resumes in Phase 4
                x = self.P4[0]
                y = self.P4[1]
                z = self.P4[2]
                grip = GRIPPER_OPEN

            else:
                # Phase 4: retract — lift straight up from P4 by lift_height
                t_ret = t - t_end_dwell
                x = self.P4[0]
                y = self.P4[1]
                z = self.P4[2] + self._lift_z(t_ret)
                grip = GRIPPER_OPEN

            self.x_path.append(x)
            self.y_path.append(y)
            self.z_path.append(z)
            self.gripper_path.append(grip)

        # Keep orientation pointing straight down throughout
        self.rot_x_path = [np.pi] * len(self.time_array)
        self.rot_y_path = [0.0]  * len(self.time_array)
        self.rot_z_path = [0.0]  * len(self.time_array)

    def _compute_timing_parameters(self):
        self.tb = self.v_max / self.a_max
        self.D  = 0.5 * self.a_max * (self.tb ** 2)

        if self.H <= 2 * self.D or self.L <= 2 * self.D:
            self.v_max = np.sqrt(self.a_max * min(self.H, self.L)) - 0.001
            self.tb = self.v_max / self.a_max
            self.D  = 0.5 * self.a_max * (self.tb ** 2)

        self.t0 = 0.0
        self.t1 = self.tb
        self.t2 = self.t1 + (self.H - 2 * self.D) / self.v_max
        self.t3 = self.t2 + self.tb
        self.t4 = self.t3 + (self.L - 2 * self.D) / self.v_max
        self.t5 = self.t4 + self.tb
        self.t6 = self.t5 + (self.H - 2 * self.D) / self.v_max
        self.t7 = self.t6 + self.tb

    def _local_x(self, t):
        """Horizontal distance along the P1→P4 direction."""
        if t < self.t2:
            return 0.0
        elif t < self.t3:
            dt = t - self.t2
            return 0.5 * self.a_max * (dt ** 2)
        elif t < self.t4:
            dt = t - self.t3
            return self.D + self.v_max * dt
        elif t < self.t5:
            dt = t - self.t4
            return (self.L - self.D) + self.v_max * dt - 0.5 * self.a_max * (dt ** 2)
        else:
            return self.L

    def _local_y(self, t):
        """Vertical (Z) displacement for the travel arc: lift up then lower down."""
        if t < self.t1:
            return 0.5 * self.a_max * (t ** 2)
        elif t < self.t2:
            dt = t - self.t1
            return self.D + self.v_max * dt
        elif t < self.t3:
            dt = t - self.t2
            return (self.H - self.D) + self.v_max * dt - 0.5 * self.a_max * (dt ** 2)
        elif t < self.t4:
            return self.H
        elif t < self.t5:
            dt = t - self.t4
            return self.H - 0.5 * self.a_max * (dt ** 2)
        elif t < self.t6:
            dt = t - self.t5
            return (self.H - self.D) - self.v_max * dt
        elif t <= self.t7:
            dt = t - self.t6
            return self.D - self.v_max * dt + 0.5 * self.a_max * (dt ** 2)
        else:
            return 0.0

    def _lift_z(self, t):
        """Trapezoidal vertical lift from z=0 to z=H (accel → cruise → decel)."""
        if t < self.t1:
            return 0.5 * self.a_max * t ** 2
        elif t < self.t2:
            dt = t - self.t1
            return self.D + self.v_max * dt
        else:
            dt = t - self.t2
            return min((self.H - self.D) + self.v_max * dt - 0.5 * self.a_max * dt ** 2,
                       self.H)


# ---------------------------------------------------------------------------
# Action Server
# ---------------------------------------------------------------------------

class CompetitionActionServer(Node):
    """
    ROS 2 Action Server that tracks LTPB trajectories using CLIK
    (Closed-Loop Inverse Kinematics) with feedforward velocity.
    """
    def __init__(self):
        super().__init__('competition_action_server')
        self.robot = RobotKinematics()

        self._action_server = ActionServer(
            self, ExecuteTrajectory, 'execute_competition', self.execute_callback)

        self._joint_cmd_pub = self.create_publisher(JointTrajectory, 'joint_cmds', 10)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self._joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, sensor_qos)

        # Control loop parameters
        self.control_rate_hz = 100.0
        self.dt = 1.0 / self.control_rate_hz
        self.K = np.diag([8.0, 8.0, 8.0, 3.0, 3.0])  # CLIK feedback gain
        self.Kp = 2.0
        self.max_vel = 0.5
        self.tolerance = 0.05

        self.current_q_arm     = None
        self.current_q_gripper = None
        self.current_ee_pose   = None
        self._control_timer    = None

    def joint_state_callback(self, msg):
        self.current_q_arm = np.array(msg.position[:5])
        if len(msg.position) > 5:
            self.current_q_gripper = msg.position[5]

    async def execute_callback(self, goal_handle):
        """
        Two-phase execution:
          Phase 1 – joint P-control moves arm to the first trajectory point.
          Phase 2 – CLIK tracks the full trajectory with feedforward velocity.
        async lets the server await completion without blocking sensor callbacks.
        """
        self.get_logger().info('Received trajectory! Preparing execution...')

        if self.current_q_arm is None:
            self.get_logger().error('Sensors not initialised. Aborting goal.')
            goal_handle.abort()
            return ExecuteTrajectory.Result()

        self._goal_handle = goal_handle

        self._t_arr       = np.array(goal_handle.request.time_array)
        self._t_final     = self._t_arr[-1]
        self._x_arr       = np.array(goal_handle.request.x_path)
        self._y_arr       = np.array(goal_handle.request.y_path)
        self._z_arr       = np.array(goal_handle.request.z_path)
        self._rot_x_arr   = np.array(goal_handle.request.rot_x_path)
        self._rot_y_arr   = np.array(goal_handle.request.rot_y_path)
        self._gripper_arr = np.array(goal_handle.request.gripper_path)

        self.Kp = 2.0
        self.max_vel = 0.5
        self.tolerance = 0.05
        self._trajectory_future = Future()

        # Compute IK for the first trajectory point
        target_pose_0 = np.array([
            self._x_arr[0], self._y_arr[0], self._z_arr[0],
            self._rot_x_arr[0], self._rot_y_arr[0]
        ])

        self.current_ee_pose = self.robot.forward_kinematics(self.current_q_arm)

        error_start = target_pose_0 - self.current_ee_pose[:5]
        error_start[3:] = (error_start[3:] + np.pi) % (2 * np.pi) - np.pi

        # Open the gripper while the arm moves to the start position so it is
        # ready to pick — the trajectory closes it during Phase 1 dwell (at P1)
        # and opens it again during Phase 3 dwell (at P4).
        init_gripper = GRIPPER_OPEN

        if np.linalg.norm(error_start) < self.tolerance:
            self.get_logger().info('Already at start position. Bypassing IK...')
            self._init_target_q_arm     = self.current_q_arm
            self._init_target_q_gripper = init_gripper
        else:
            self.get_logger().info('Computing IK for start position...')
            ik_solutions = self.robot.inverse_kinematics(target_pose_0.tolist(), n_restarts=15)

            if not ik_solutions:
                self.get_logger().error(f'Start position unreachable: {target_pose_0}')
                goal_handle.abort()
                return ExecuteTrajectory.Result()

            self._init_target_q_arm = min(
                ik_solutions,
                key=lambda q: np.linalg.norm(np.array(q) - self.current_q_arm))
            self._init_target_q_gripper = init_gripper

        self._tracking_errors = []
        self._tracking_times  = []

        self.get_logger().info('Moving to start position...')
        self._init_timer = self.create_timer(self.dt, self.init_control_loop)

        await self._trajectory_future

        goal_handle.succeed()
        result = ExecuteTrajectory.Result()
        result.success = True
        result.message = 'Trajectory completed successfully.'
        return result

    def init_control_loop(self):
        """Phase 1 – joint P-control to reach the trajectory's first point."""
        error_arm     = np.array(self._init_target_q_arm) - self.current_q_arm
        error_gripper = self._init_target_q_gripper - self.current_q_gripper

        if np.linalg.norm(error_arm) < self.tolerance and abs(error_gripper) < self.tolerance:
            self.get_logger().info('Start position reached! Launching CLIK trajectory...')
            self._init_timer.cancel()
            self.destroy_timer(self._init_timer)
            self._start_time = self.get_clock().now().nanoseconds / 1e9
            self._control_timer = self.create_timer(self.dt, self.timer_control_loop)
            return

        v_cmd_arm     = np.clip(self.Kp * error_arm,     -self.max_vel, self.max_vel)
        v_cmd_gripper = np.clip(self.Kp * error_gripper, -self.max_vel, self.max_vel)
        self.publish_velocities(np.append(v_cmd_arm, v_cmd_gripper))

    def timer_control_loop(self):
        """Phase 2 – CLIK tracking via Jacobian inverse + feedforward velocity."""
        current_time = (self.get_clock().now().nanoseconds / 1e9) - self._start_time

        self.current_ee_pose = self.robot.forward_kinematics(self.current_q_arm)

        if current_time >= self._t_final:
            self.get_logger().info('All trajectory points reached. Stopping.')
            self._control_timer.cancel()
            self.publish_velocities(np.zeros(6))

            errors_array = np.array(self._tracking_errors)
            if len(errors_array) > 0:
                rmse_pos = np.sqrt(np.mean(errors_array[:, 0] ** 2))
                max_pos  = np.max(errors_array[:, 0])
                rmse_rot = np.sqrt(np.mean(errors_array[:, 1] ** 2))
                max_rot  = np.max(errors_array[:, 1])
                self.get_logger().info('\n=== Trajectory Tracking Performance ===')
                self.get_logger().info(f'Position RMSE: {rmse_pos:.5f} m  |  Max: {max_pos:.5f} m')
                self.get_logger().info(f'Rotation RMSE: {rmse_rot:.5f} rad  |  Max: {max_rot:.5f} rad')
                self.get_logger().info('========================================\n')

            self._trajectory_future.set_result(True)
            return

        # Interpolate desired pose at current time
        desired_pose = np.array([
            np.interp(current_time, self._t_arr, self._x_arr),
            np.interp(current_time, self._t_arr, self._y_arr),
            np.interp(current_time, self._t_arr, self._z_arr),
            np.interp(current_time, self._t_arr, self._rot_x_arr),
            np.interp(current_time, self._t_arr, self._rot_y_arr),
        ])
        target_gripper = np.interp(current_time, self._t_arr, self._gripper_arr)

        actual_pose = self.current_ee_pose[:5]
        error = desired_pose - actual_pose
        error[3:] = (error[3:] + np.pi) % (2 * np.pi) - np.pi

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

        # Clip joint velocities — near-singular configurations cause J_inv to
        # amplify commands; without this clip, large velocities can occur
        q_dot_arm = np.clip(q_dot_arm, -self.max_vel, self.max_vel)

        # Enforce joint limits
        for i in range(5):
            low, high = self.robot.joint_bounds[i]
            if self.current_q_arm[i] >= high and q_dot_arm[i] > 0:
                self.get_logger().warn(
                    f'Joint {i} hit UPPER limit! Velocity zeroed.', throttle_duration_sec=0.5)
                q_dot_arm[i] = 0.0
            elif self.current_q_arm[i] <= low and q_dot_arm[i] < 0:
                self.get_logger().warn(
                    f'Joint {i} hit LOWER limit! Velocity zeroed.', throttle_duration_sec=0.5)
                q_dot_arm[i] = 0.0

        # Gripper P-control
        error_gripper = target_gripper - self.current_q_gripper
        v_gripper = np.clip(self.Kp * error_gripper, -self.max_vel, self.max_vel)

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
# Action Client
# ---------------------------------------------------------------------------

class CompetitionActionClient(Node):
    """
    ROS 2 Action Client.
    Reads Cartesian pick/place positions from the module-level constants,
    generates LTPB trajectories, and sends them to the server.
    """
    def __init__(self):
        super().__init__('competition_action_client')
        self._action_client = ActionClient(self, ExecuteTrajectory, 'execute_competition')
        self.action_result  = None
        self.is_action_done = False

        self.P_PICK   = PICK
        self.P_PLACE1 = PLACE1
        self.P_PLACE2 = PLACE2
        self.P_PLACE3 = PLACE3

        self.get_logger().info(f'Pick:   {self.P_PICK}')
        self.get_logger().info(f'Place1: {self.P_PLACE1}')
        self.get_logger().info(f'Place2: {self.P_PLACE2}')
        self.get_logger().info(f'Place3: {self.P_PLACE3}')

    def send_trajectory_goal(self, p_start, p_end, height, grasp=True):
        """Generate an LTPB trajectory and send it as an action goal."""
        self.is_action_done = False
        self.action_result  = None

        self.get_logger().info('Generating LTPB trajectory...')

        traj = LTPBTrajectory(P1=p_start, P4=p_end, lift_height=height,
                              grasp=grasp, v_max=0.1, a_max=0.2)

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
        time.sleep(0.5)  # brief pause between sequential goals for smooth handoff

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        pass

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server.')
            self.is_action_done = True
            return
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
    node = CompetitionActionServer()
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
    node = CompetitionActionClient()

    # 3 pick-and-place cycles: pick from the same spot, place at 3 locations
    sequence = [
        {'name': 'PICK → PLACE1', 'start': node.P_PICK, 'end': node.P_PLACE1, 'height': LIFT_HEIGHT, 'grasp': True},
        {'name': 'PICK → PLACE2', 'start': node.P_PICK, 'end': node.P_PLACE2, 'height': LIFT_HEIGHT, 'grasp': True},
        {'name': 'PICK → PLACE3', 'start': node.P_PICK, 'end': node.P_PLACE3, 'height': LIFT_HEIGHT, 'grasp': True},
    ]

    for step in sequence:
        node.get_logger().info(f'\n--- Starting: {step["name"]} ---')
        node.send_trajectory_goal(
            p_start=step['start'],
            p_end=step['end'],
            height=step['height'],
            grasp=step['grasp']
        )

        while rclpy.ok() and not node.is_action_done:
            rclpy.spin_once(node)

        if not (node.action_result and node.action_result.success):
            node.get_logger().error(f'Failed during {step["name"]}. Aborting mission.')
            break

    node.get_logger().info('Competition sequence complete. Shutting down client.')
    node.destroy_node()
    rclpy.shutdown()


# Default entry point runs the server
def main(args=None):
    main_server(args)


if __name__ == '__main__':
    main()
