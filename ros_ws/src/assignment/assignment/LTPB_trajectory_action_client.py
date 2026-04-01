import sys
import os

_ASSIGNMENT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_PROJECT_ROOT   = os.path.dirname(_ASSIGNMENT_DIR)
sys.path.insert(0, _ASSIGNMENT_DIR)
sys.path.insert(0, os.path.join(_PROJECT_ROOT, 'ros_ws', 'src', 'edubot_interfaces'))

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from my_robot_msgs.action import ExecuteTrajectory
import time

# --- DEFINE KEY POSITIONS ---
HOME = [0.15, 0.15, 0.05] # A safe, neutral resting position
P1   = [0.10, 0.15, 0.05] # Pick/Drop point 1 
P4   = [0.20, 0.15, 0.05] # Pick/Drop point 2

# Dictionary defining the sequence of movements using our key positions
PICK_AND_PLACE_POINTS = {
    'HOME_TO_P1': {'P1': HOME, 'P4': P1,   'height': 0.05, 'grasp': False},
    'P1_TO_P4':   {'P1': P1,   'P4': P4,   'height': 0.05, 'grasp': True},
    'P4_TO_HOME': {'P1': P4,   'P4': HOME, 'height': 0.05, 'grasp': False},
    'HOME_TO_P4': {'P1': HOME, 'P4': P4,   'height': 0.05, 'grasp': False},
    'P4_TO_P1':   {'P1': P4,   'P4': P1,   'height': 0.05, 'grasp': True},
    'P1_TO_HOME': {'P1': P1,   'P4': HOME, 'height': 0.05, 'grasp': False}
}

class TrajectoryPlannerNode(Node):
    """
    ROS 2 Action Client Node.
    Responsible for generating the trajectory arrays and sending them to the Action Server.
    """
    def __init__(self):
        super().__init__('trajectory_planner_client')
        self._action_client = ActionClient(self, ExecuteTrajectory, 'execute_pick_and_place')

        self.action_result = None
        self.is_action_done = False

    def send_trajectory_goal(self, p_start, p_end, height, grasp=True):
        """Generates the trajectory math and sends the Goal request to the server."""
        self.is_action_done = False
        self.action_result = None
        
        self.get_logger().info('Generating Trajectory...')
        
        # 1. Generate the smooth math trajectory (LTPB)
        traj = LTPB_Trajectory(P1=p_start, P4=p_end, lift_height=height, grasp=grasp, v_max=0.1, a_max=0.2)
        
        # 2. Pack the generated arrays into the custom ROS 2 Action Goal message
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.time_array = traj.time_array.tolist()
        goal_msg.x_path = traj.x_path
        goal_msg.y_path = traj.y_path
        goal_msg.z_path = traj.z_path
        goal_msg.rot_x_path = traj.rot_x_path
        goal_msg.rot_y_path = traj.rot_y_path
        goal_msg.rot_z_path = traj.rot_z_path
        goal_msg.gripper_path = traj.gripper_path   

        # 3. Wait for the server to be online before sending
        self._action_client.wait_for_server()
        time.sleep(0.5) # Brief pause between sequential commands to ensure smooth handoffs
        
        # 4. Send the goal ASYNCHRONOUSLY. 
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        # Optional: Print feedback if desired
        pass

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by Controller.')
            self.is_action_done = True 
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.action_result = future.result().result
        self.is_action_done = True 
        self.get_logger().info(f'Result: {self.action_result.message}')


class LTPB_Trajectory:
    """
    Linear Trajectory with Parabolic Blends (LTPB).
    Creates a smooth profile ensuring the robot accelerates and decelerates cleanly.
    Now includes a "dwell time" to let the gripper fully open or close!
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

        if self.L == 0:
            self.theta = 0.0
        else: 
            self.theta = np.arctan2(self.dy, self.dx)

        self.cos_theta = np.cos(self.theta)
        self.sin_theta = np.sin(self.theta)
        
        self._compute_timing_parameters()
        
        # --- NEW: Gripper Dwell Time ---
        # At max_vel = 0.5 rad/s, moving 0.4 rad takes 0.8 seconds. 
        # We give it 1.0 second to be perfectly safe.
        self.t_grip = 1.0  
        
        # Total trajectory time now includes the wait at the start and the end
        total_time = self.t7 + (2 * self.t_grip)
        self.time_array = np.linspace(0, total_time, 150)
        
        self.x_path = []
        self.y_path = []
        self.z_path = []
        self.gripper_path = []
        
        for t in self.time_array:
            if t < self.t_grip:
                # PHASE 1: Wait at P1, actuate gripper
                ltpb_t = 0.0
                grip_val = np.pi/50 if grasp else 0.4
            elif t < self.t_grip + self.t7:
                # PHASE 2: Move arm through trajectory, hold gripper state
                ltpb_t = t - self.t_grip
                grip_val = np.pi/50 if grasp else 0.4
            else:
                # PHASE 3: Arrive at P4, release gripper
                ltpb_t = self.t7
                grip_val = 0.4 
            
            self.x_path.append(self.P1[0] + self.get_local_x(ltpb_t) * self.cos_theta)
            self.y_path.append(self.P1[1] + self.get_local_x(ltpb_t) * self.sin_theta)
            self.z_path.append(self.P1[2] + self.get_local_y(ltpb_t))
            self.gripper_path.append(grip_val)

        # Orientation is kept constant pointing straight down
        self.rot_x_path = [np.pi] * len(self.time_array)
        self.rot_y_path = [0.0] * len(self.time_array)
        self.rot_z_path = [0.0] * len(self.time_array)

    def _compute_timing_parameters(self):
        self.tb = self.v_max / self.a_max 
        self.D = 0.5 * self.a_max * (self.tb ** 2) 
        
        if self.H <= 2 * self.D or self.L <= 2 * self.D:
            self.v_max = np.sqrt(self.a_max * min(self.H, self.L)) - 0.001
            self.tb = self.v_max / self.a_max
            self.D = 0.5 * self.a_max * (self.tb ** 2)

        self.t0 = 0.0
        self.t1 = self.tb                                       
        self.t2 = self.t1 + (self.H - 2 * self.D) / self.v_max  
        self.t3 = self.t2 + self.tb                             
        self.t4 = self.t3 + (self.L - 2 * self.D) / self.v_max  
        self.t5 = self.t4 + self.tb                             
        self.t6 = self.t5 + (self.H - 2 * self.D) / self.v_max  
        self.t7 = self.t6 + self.tb                             
    
    def get_local_x(self, t):
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
            return (self.L - self.D) + (self.v_max * dt) - (0.5 * self.a_max * (dt ** 2))
        else:
            return self.L
    
    def get_local_y(self,t):
        if t < self.t1:   
            return 0.5 * self.a_max * (t ** 2)
        elif t < self.t2: 
            dt = t - self.t1
            return self.D + self.v_max * dt
        elif t < self.t3: 
            dt = t - self.t2
            return (self.H - self.D) + (self.v_max * dt) - (0.5 * self.a_max * (dt ** 2))
        elif t < self.t4: 
            return self.H
        elif t < self.t5: 
            dt = t - self.t4
            return self.H - (0.5 * self.a_max * (dt ** 2))
        elif t < self.t6: 
            dt = t - self.t5
            return (self.H - self.D) - (self.v_max * dt)
        elif t <= self.t7: 
            dt = t - self.t6
            return self.D - (self.v_max * dt) + (0.5 * self.a_max * (dt ** 2))
        else:
            return 0.0

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlannerNode()
    
    # Define the execution order
    execution_sequence = [
        'HOME_TO_P1',
        'P1_TO_P4',
        'P4_TO_HOME',
        'HOME_TO_P4',
        'P4_TO_P1',
        'P1_TO_HOME'
    ]

    for step_name in execution_sequence:
        movement = PICK_AND_PLACE_POINTS[step_name]
        node.get_logger().info(f'\n--- Starting Phase: {step_name} ---')
        
        node.send_trajectory_goal(
            p_start=movement['P1'], 
            p_end=movement['P4'], 
            height=movement['height'], 
            grasp=movement['grasp']
        )

        # Block script until the current trajectory phase is complete
        while rclpy.ok() and not node.is_action_done:
            rclpy.spin_once(node)
            
        # Verify success before moving to the next sequence item
        if not (node.action_result and node.action_result.success):
            node.get_logger().error(f'Failed during {step_name}. Aborting mission.')
            break
            
    node.get_logger().info('Full cycle complete! Shutting down client.')
       
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()