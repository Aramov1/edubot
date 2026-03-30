import sys
import os

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.task import Future            
import numpy as np

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from edubot_interfaces.action import ExecuteTrajectory
from .robot_kinematics import RobotKinematics

def quaternion_to_euler(x, y, z, w):
    """Convert quaternion to roll, pitch, yaw in degrees"""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)
    
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw 

class TrajectoryControllerNode(Node):
    def __init__(self):
        super().__init__('trajectory_controller_action_server') # initialize action server node
        self.robot = RobotKinematics() # call robot kinematics class for IK and Jacobian functions
        
        # initialize action server with the ExecuteTrajectory action and the execute_callback function
        self._action_server = ActionServer(
            self, ExecuteTrajectory, 'execute_pick_and_place', self.execute_callback) 
        
        # initialize publisher for joint commands
        self._joint_cmd_pub = self.create_publisher(JointTrajectory, 'joint_cmds', 10)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        # initialize subscribers for joint positions and end-effector pose
        self._joint_state_sub = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, sensor_qos)

        # Control Loop Parameters
        self.control_rate_hz = 100.0  
        self.dt = 1.0 / self.control_rate_hz
        self.K = np.diag([8.0, 8.0, 8.0, 3.0, 3.0])
        
        # Initialize joint values (arm + gripper), end-effector pose, and control timer
        self.current_q_arm = None
        self.current_q_gripper = None
        self.current_ee_pose = None
        self._control_timer = None

    def joint_state_callback(self, msg):
        # Read arm and gripper separately and assign to current_q_arm and current_q_gripper
        self.current_q_arm = np.array(msg.position[:5])
        if len(msg.position) > 5:
            self.current_q_gripper = msg.position[5]
    


    async def execute_callback(self, goal_handle):
        """
        This function is called when a new goal is received by the action server. 
        It processes the incoming trajectory, initializes the control loop, 
        and manages the execution of the trajectory while providing feedback to the client.
        Using 'async' allows the server to wait for the trajectory to complete without 
        blocking its execution thread, ensuring it can continue processing sensor callbacks, 
        running the control loop timers, and handling potential preemptions.
        """
        # Confirmation trajectory has been received 
        self.get_logger().info('Received trajectory! Preparing execution...')  

        # Sanity check to ensure sensor data is available before starting the trajectory execution
        if self.current_q_arm is None:
            self.get_logger().error('Sensors not initialized. Aborting goal.')
            goal_handle.abort()
            return ExecuteTrajectory.Result()
            
        self._goal_handle = goal_handle
        
        # Get time array and trajectory points from the goal message and store them as numpy arrays for easy interpolation
        self._t_arr = np.array(goal_handle.request.time_array)
        self._t_final = self._t_arr[-1]
        
        self._x_arr = np.array(goal_handle.request.x_path)
        self._y_arr = np.array(goal_handle.request.y_path)
        self._z_arr = np.array(goal_handle.request.z_path)
        self._rot_x_arr = np.array(goal_handle.request.rot_x_path)
        self._rot_y_arr = np.array(goal_handle.request.rot_y_path)
        self._rot_z_arr = np.array(goal_handle.request.rot_z_path)
        self._gripper_arr = np.array(goal_handle.request.gripper_path)
        
        # Max angular velocity allowed, gain for P-control, tolerance for reaching points, and a Future to signal when the trajectory is complete
        self.Kp = 1.5
        self.max_vel = 0.5
        self.tolerance = 0.05
        self._trajectory_future = Future() 
        
        # Extract the very first point of the trajectory
        target_pose_0 = np.array([
            self._x_arr[0], self._y_arr[0], self._z_arr[0],
            self._rot_x_arr[0], self._rot_y_arr[0]
        ])

        self.current_ee_pose = self.robot.forward_kinematics(self.current_q_arm)
        
        # Get current EE pose, get error to first point traj, keep angles wrapped to (−π, π) 
        error_start = target_pose_0 - self.current_ee_pose[:5]
        error_start[3:] = (error_start[3:] + np.pi) % (2 * np.pi) - np.pi
        
        # If we are already within tolerance, skip the IK math, and assign the current joint angles!
        if np.linalg.norm(error_start) < self.tolerance:
            self.get_logger().info('Already at starting position! Bypassing IK...')
            self._init_target_q_arm = self.current_q_arm
            self._init_target_q_gripper = self._gripper_arr[0]
        else:
            self.get_logger().info('Computing IK for the starting position...')
            ik_solutions = self.robot.inverse_kinematics(target_pose_0.tolist(), n_restarts=15)
            
            if not ik_solutions:
                self.get_logger().error(f"Starting position unreachable: {target_pose_0}")
                goal_handle.abort()
                return ExecuteTrajectory.Result()
            
            # Choose IK solution closest to current joint angles so that it is smooth
            self._init_target_q_arm = min(ik_solutions, key=lambda q: np.linalg.norm(np.array(q) - self.current_q_arm))
            self._init_target_q_gripper = self._gripper_arr[0]
        
        self._tracking_errors = []
        self._tracking_times = []

        # Start the Initialization Timer and go to starting position using P-Control 
        self.get_logger().info('Moving to starting position...')
        self._init_timer = self.create_timer(self.dt, self.init_control_loop)
        
        # This will not go through until the trajectory is complete and self._trajectory_future 
        # is set to True in timer_control_loop()
        await self._trajectory_future 
        
        # Once trajectory is complete it returns a success message to the client
        goal_handle.succeed() 
        result = ExecuteTrajectory.Result()
        result.success = True
        result.message = "Trajectory completed smoothly."
        return result
    
    def init_control_loop(self):
        """Phase 1: Safely move to the very first point of the trajectory using Joint P-Control."""
        error_arm = np.array(self._init_target_q_arm) - self.current_q_arm
        error_gripper = self._init_target_q_gripper - self.current_q_gripper
        
        # Check if BOTH arm and gripper reached the start position
        if np.linalg.norm(error_arm) < self.tolerance and abs(error_gripper) < self.tolerance:
            self.get_logger().info('Start position reached! Launching CLIK trajectory...')
            
            # Kill this initialization timer
            self._init_timer.cancel()
            self.destroy_timer(self._init_timer)
            
            # Reset the time exactly as the trajectory begins!
            self._start_time = self.get_clock().now().nanoseconds / 1e9
            
            # Spawn the CLIK timer to take over
            self._control_timer = self.create_timer(self.dt, self.timer_control_loop)
            return

        # P-Control Math for Arm, and clip to max angular velocity limits
        v_cmd_arm = self.Kp * error_arm
        v_cmd_arm = np.clip(v_cmd_arm, -self.max_vel, self.max_vel)
        
        # P-Control Math for Gripper, and clip to max angular velocity limits
        v_cmd_gripper = self.Kp * error_gripper
        v_cmd_gripper = np.clip(v_cmd_gripper, -self.max_vel, self.max_vel)
        
        # Stitch and Publish
        v_cmd_full = np.append(v_cmd_arm, v_cmd_gripper)
        self.publish_velocities(v_cmd_full)
    
    def position_control_loop(self):
        """
        The position control loop uses the current joint angles to compute the end-effector pose, 
        compares it to the desired pose from the trajectory, and computes velocity commands 
        using P-Control for smooth trajectory tracking. It also handles the transition between 
        waypoints and provides feedback to the client. 
        This controller should be runned when using simple trajectory in action_client, 
        and not the LTPB trajectory which uses inverse Jacobian control in timer_control_loop()
        """
        if self._current_idx >= self._num_points:
            self.get_logger().info('All trajectory points reached. Stopping.')
            self._control_timer.cancel()
            self.publish_velocities(np.zeros(6))
            self._trajectory_future.set_result(True) 
            return

        if self._current_target_q_arm is None:
            # Drop self._rot_z_arr[self._current_idx]
            target_pose = [
                self._x_arr[self._current_idx], self._y_arr[self._current_idx], self._z_arr[self._current_idx],
                self._rot_x_arr[self._current_idx], self._rot_y_arr[self._current_idx]
            ]
            self.get_logger().info(f'Computing IK for waypoint {self._current_idx + 1}/{self._num_points}...')
            
            
            ik_solutions = self.robot.inverse_kinematics(target_pose, n_restarts=15)

            if not ik_solutions:
                self.get_logger().error(f"Waypoint {self._current_idx} unreachable: {target_pose}")
                self._goal_handle.abort()
                self._control_timer.cancel()
                self.destroy_timer(self._control_timer)
                self._trajectory_future.set_result(False)
                return
                
            self._current_target_q_arm = min(ik_solutions, key=lambda q: np.linalg.norm(np.array(q) - self.current_q_arm))

            self.target_q_gripper = self._gripper_arr[self._current_idx]

        # Calculate Error for both Arm and Gripper
        error_arm = np.array(self._current_target_q_arm) - self.current_q_arm
        error_gripper = self.target_q_gripper - self.current_q_gripper
        
        # Check if BOTH arm and gripper reached their targets
        if np.linalg.norm(error_arm) < self.tolerance and abs(error_gripper) < self.tolerance:
            self.get_logger().info(f'Reached waypoint {self._current_idx + 1}.')
            self._current_idx += 1
            self._current_target_q_arm = None 
            
            feedback_msg = ExecuteTrajectory.Feedback()
            feedback_msg.percent_complete = (self._current_idx / self._num_points) * 100.0
            self._goal_handle.publish_feedback(feedback_msg)
            return

        # P-Control Math for Arm
        v_cmd_arm = self.Kp * error_arm
        v_cmd_arm = np.clip(v_cmd_arm, -self.max_vel, self.max_vel)
        
        # P-Control Math for Gripper
        v_cmd_gripper = self.Kp * error_gripper
        v_cmd_gripper = np.clip(v_cmd_gripper, -self.max_vel, self.max_vel)
        
        # Stitch
        v_cmd_full = np.append(v_cmd_arm, v_cmd_gripper)
        self.publish_velocities(v_cmd_full)

    def publish_velocities(self, velocities):
        """Helper function to cleanly publish velocity commands."""
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.robot.joint_names + ['Gripper']
        
        point = JointTrajectoryPoint()
        point.velocities = list(velocities)
        point.time_from_start = rclpy.time.Duration(seconds=0.0).to_msg()
        
        traj_msg.points.append(point)
        self._joint_cmd_pub.publish(traj_msg)

    def timer_control_loop(self):
        # Update current time 
        current_time = (self.get_clock().now().nanoseconds / 1e9) - self._start_time

        self.current_ee_pose = self.robot.forward_kinematics(self.current_q_arm)
        
        # Check if we have reached the end of the trajectory
        if current_time >= self._t_final:
            self.get_logger().info('All trajectory points reached. Stopping.')
            self._control_timer.cancel()
            self.publish_velocities(np.zeros(6))

            # Show tracking performance stats
            errors_array = np.array(self._tracking_errors)
            if len(errors_array) > 0:
                rmse_pos = np.sqrt(np.mean(errors_array[:, 0]**2))
                max_pos_err = np.max(errors_array[:, 0])
                rmse_rot = np.sqrt(np.mean(errors_array[:, 1]**2))
                max_rot_err = np.max(errors_array[:, 1])
                
                self.get_logger().info('\n=== Trajectory Tracking Performance ===')
                self.get_logger().info(f'Position RMSE: {rmse_pos:.5f} m  |  Max Error: {max_pos_err:.5f} m')
                self.get_logger().info(f'Rotation RMSE: {rmse_rot:.5f} rad  |  Max Error: {max_rot_err:.5f} rad')
                self.get_logger().info('=======================================\n')

            self._trajectory_future.set_result(True) 
            return
            
        # Interpolate all 5 targets + gripper
        target_x = np.interp(current_time, self._t_arr, self._x_arr)
        target_y = np.interp(current_time, self._t_arr, self._y_arr)
        target_z = np.interp(current_time, self._t_arr, self._z_arr)
        target_rot_x = np.interp(current_time, self._t_arr, self._rot_x_arr)
        target_rot_y = np.interp(current_time, self._t_arr, self._rot_y_arr)
        target_gripper = np.interp(current_time, self._t_arr, self._gripper_arr)
        
        # Store target pose and current pose, compute error, and wrap angles to (−π, π)
        desired_pose = np.array([target_x, target_y, target_z, target_rot_x, target_rot_y])
        actual_pose = self.current_ee_pose[:5] 
        error = desired_pose - actual_pose
        error[3:] = (error[3:] + np.pi) % (2 * np.pi) - np.pi

        pos_error_norm = np.linalg.norm(error[:3]) # X, Y, Z error
        rot_error_norm = np.linalg.norm(error[3:]) # Roll, Pitch error
        # self.get_logger().info(f'Time: {current_time:.2f}s | Pos Error: {pos_error_norm:.4f} m | Rot Error: {rot_error_norm:.4f} rad')
        
        self._tracking_errors.append((pos_error_norm, rot_error_norm))
        self._tracking_times.append(current_time)
        
        # We need the previous target pose to compute ff velocity, so we interpolate it as well. 
        target_x_prev = np.interp(current_time - self.dt, self._t_arr, self._x_arr)
        target_y_prev = np.interp(current_time - self.dt, self._t_arr, self._y_arr)
        target_z_prev = np.interp(current_time - self.dt, self._t_arr, self._z_arr)
        target_rot_x_prev = np.interp(current_time - self.dt, self._t_arr, self._rot_x_arr)
        target_rot_y_prev = np.interp(current_time - self.dt, self._t_arr, self._rot_y_arr)
        
        # Compute v_ff using finite difference of desired pose
        prev_pose = np.array([target_x_prev, target_y_prev, target_z_prev, target_rot_x_prev, target_rot_y_prev])
        v_ff = (desired_pose - prev_pose) / self.dt 
        
        # --- ARM: 5D Inverse Jacobian ---
        
        J_inv, is_sing = self.robot.jacobian_inverse(self.current_q_arm) 
        
        q_dot_arm = J_inv @ (v_ff + self.K @ error)

        # Enforce Joint Limits on the COMMANDS, not the sensors
        for i in range(5):
            low, high = self.robot.joint_bounds[i]
            
            # If at upper limit and trying to go higher, cut the throttle
            if self.current_q_arm[i] >= high and q_dot_arm[i] > 0:
                q_dot_arm[i] = 0.0
            
            # If at lower limit and trying to go lower, cut the throttle
            elif self.current_q_arm[i] <= low and q_dot_arm[i] < 0:
                q_dot_arm[i] = 0.0
        # -----------------------------------
        
        # --- GRIPPER: Simple P-Control ---
        error_gripper = target_gripper - self.current_q_gripper
        v_gripper = self.Kp * error_gripper
        v_gripper = np.clip(v_gripper, -self.max_vel, self.max_vel)
        
        # Stitch and Publish!
        v_cmd_full = np.append(q_dot_arm, v_gripper)
        self.publish_velocities(v_cmd_full)
        
        feedback_msg = ExecuteTrajectory.Feedback()
        feedback_msg.current_time = current_time
        feedback_msg.percent_complete = (current_time / self._t_final) * 100.0
        self._goal_handle.publish_feedback(feedback_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryControllerNode()
    
    # We can go back to the standard spin! No MultiThreadedExecutor needed.
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()