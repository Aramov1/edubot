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

# Pre-defined coordinates for the pick-and-place task.
# P1 is the starting/pick location, P4 is the drop-off location.
PICK_AND_PLACE_POINTS = {
    'MOVEMENT_1': {
        'P1': [0.0, 0.3, 0.05],
        'P4': [0.25, 0.0, 0.05],
        'height': 0.06
    },
    'MOVEMENT_2': {
        'P1': [0.25, 0.0, 0.05],
        'P4': [0.0, 0.3, 0.05],
        'height': 0.06 
    },
    'MOVEMENT_3': {
        'P1': [0.0, 0.05, 0.0],
        'P4': [0.2, 0.05, 0.0],
        'height': 0.1
    },
    'MOVEMENT_4': {
        'P1': [0.2, 0.05, 0.0],
        'P4': [0.0, 0.05, 0.0],
        'height': 0.1
    }
}

class TrajectoryPlannerNode(Node):
    """
    ROS 2 Action Client Node.
    Responsible for generating the trajectory arrays and sending them to the Action Server.
    """
    def __init__(self):
        super().__init__('trajectory_planner_client')
        # Create an Action Client to communicate with the 'execute_pick_and_place' server
        self._action_client = ActionClient(self, ExecuteTrajectory, 'execute_pick_and_place')

        # State flags to help our main loop know when an action is fully complete
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
        self.get_logger().info('Server found! Giving sensors 1 second to initialize...')
        time.sleep(1.0)
        self.get_logger().info('Sending Goal to Controller...')
        
        # 4. Send the goal ASYNCHRONOUSLY. 
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        
        # Attach a callback for when the server responds with "Accepted" or "Rejected"
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        """Called repeatedly while the server is executing the trajectory."""
        feedback = feedback_msg.feedback
        # Uncomment the line below to see live progress percentage in the terminal
        # self.get_logger().info(f'Robot moving: {feedback.percent_complete:.1f}% complete')

    def goal_response_callback(self, future):
        """Called exactly once when the server decides to accept or reject the initial goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by Controller.')
            self.is_action_done = True # Mark as done so the main loop doesn't hang forever waiting
            return

        self.get_logger().info('Goal accepted! Executing...')
        
        # If accepted, we now ask for the FINAL result, which will be delivered later
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Called exactly once when the server has completely finished the task."""
        self.action_result = future.result().result
        self.is_action_done = True # The action is fully complete! This wakes up the while loop in main()!
        self.get_logger().info(f'Result: {self.action_result.message}')


class SimpleTrajectory:
    """
    A naive, blocky trajectory generator. 
    It moves in straight lines with no smooth acceleration/deceleration.
    (Included for comparison, but LTPB is used in the final implementation).
    """
    def __init__(self, P1, P4, lift_height, grasp=True):
        self.P1 = np.array(P1)
        self.P4 = np.array(P4)
        self.lift_height = lift_height
        self.H = self.lift_height

        self.x_path = [self.P1[0], self.P1[0], self.P1[0], self.P4[0], self.P4[0], self.P4[0]]
        self.y_path = [self.P1[1], self.P1[1], self.P1[1], self.P4[1], self.P4[1], self.P4[1]]
        self.z_path = [self.P1[2], self.P1[2], self.P1[2]+self.H, self.P1[2]+self.H, self.P1[2], self.P1[2]]

        # Dynamic Gripper Logic
        if grasp:
            # Standard Pick and Place: Open, close, move, open.
            self.gripper_path = [0.4, 0.0, 0.0, 0.0, 0.0, 0.4]
        else:
            # Empty return trip: stays open (0.4) the entire time
            self.gripper_path = [0.4, 0.4, 0.4, 0.4, 0.4, 0.4]

class LTPB_Trajectory:
    """
    Linear Trajectory with Parabolic Blends (LTPB).
    Creates a smooth profile ensuring the robot accelerates and decelerates cleanly
    rather than jerking instantly to max velocity.
    """
    def __init__(self, P1, P4, lift_height, v_max=0.1, a_max=0.2, grasp=True):
        self.P1 = np.array(P1)
        self.P4 = np.array(P4)
        self.lift_height = lift_height
        self.v_max = v_max
        self.a_max = a_max

        # Calculate total distance to travel in the XY plane
        self.dx = self.P4[0] - self.P1[0]
        self.dy = self.P4[1] - self.P1[1]
        self.L = np.sqrt(self.dx**2 + self.dy**2)
        self.H = self.lift_height

        # Calculate the angle of travel in the XY plane
        if self.L == 0:
            self.theta = 0.0
        else: 
            self.theta = np.arctan2(self.dy, self.dx)

        self.cos_theta = np.cos(self.theta)
        self.sin_theta = np.sin(self.theta)
        
        # Pre-compute the time segments for acceleration/cruise/deceleration
        self._compute_timing_parameters()
        
        # Generate 120 points evenly spaced across the total trajectory time
        self.time_array = np.linspace(0, self.t7 + 0.5, 120)
        
        # Build the X, Y, Z paths by calculating the position at every specific time 't'
        self.x_path = [self.P1[0] + self.get_local_x(t)*self.cos_theta for t in self.time_array]
        self.y_path = [self.P1[1] + self.get_local_x(t)*self.sin_theta for t in self.time_array]
        self.z_path = [self.P1[2] + self.get_local_y(t) for t in self.time_array]

        # Orientation is kept constant (pointing straight down)
        self.rot_x_path = [np.pi] * len(self.time_array)
        self.rot_y_path = [0.0] * len(self.time_array)
        self.rot_z_path = [0.0] * len(self.time_array)
        
        # Gripper Logic: Close immediately (0.0), and open (0.4) once the movement finishes (t7)
        if grasp:
            self.gripper_path = [0.0 if t < self.t7 else 0.4 for t in self.time_array]
        else:
            self.gripper_path = [0.4] * len(self.time_array)

    def _compute_timing_parameters(self):
        """
        Calculates the exact timestamps for the 7 phases of a pick-and-place arc.
        t1, t2: Lifting up
        t3, t4: Moving across the table
        t5, t6: Placing down
        """
        self.tb = self.v_max / self.a_max # Blend time (time spent accelerating)
        self.D = 0.5 * self.a_max * (self.tb ** 2) # Distance covered during acceleration
        
        # Safety check: We can't reach max speed if the distance is too short
        if self.H <= 2 * self.D or self.L <= 2 * self.D:
            raise ValueError("Height or Length is too short to reach v_max with this a_max.")
        
        self.t0 = 0.0
        self.t1 = self.tb                                       # End of Lift Accel
        self.t2 = self.t1 + (self.H - 2 * self.D) / self.v_max  # End of Lift Cruise
        self.t3 = self.t2 + self.tb                             # End of Lift Decel / Start Traverse Accel
        self.t4 = self.t3 + (self.L - 2 * self.D) / self.v_max  # End of Traverse Cruise
        self.t5 = self.t4 + self.tb                             # End of Traverse Decel / Start Drop Accel
        self.t6 = self.t5 + (self.H - 2 * self.D) / self.v_max  # End of Drop Cruise
        self.t7 = self.t6 + self.tb                             # End of Drop Decel (Task Complete)
    
    def get_local_x(self, t):
        """Piecewise function defining the horizontal distance traveled over time."""
        if t < self.t2:
            return 0.0
        elif t < self.t3: # Phase 3: Corner 1 (Accelerate horizontally)
            dt = t - self.t2
            return 0.5 * self.a_max * (dt ** 2)
        elif t < self.t4: # Phase 4: Traverse (Cruise horizontally at max speed)
            dt = t - self.t3
            return self.D + self.v_max * dt
        elif t < self.t5: # Phase 5: Corner 2 (Decelerate horizontally)
            dt = t - self.t4
            return (self.L - self.D) + (self.v_max * dt) - (0.5 * self.a_max * (dt ** 2))
        else:
            return self.L
    
    def get_local_y(self,t):
        """Piecewise function defining the vertical lifting distance over time."""
        if t < self.t1:   # Phase 1: Lift (Accelerate upwards)
            return 0.5 * self.a_max * (t ** 2)
        elif t < self.t2: # Phase 2: Lift (Cruise upwards at max speed)
            dt = t - self.t1
            return self.D + self.v_max * dt
        elif t < self.t3: # Phase 3: Corner 1 (Decelerate upwards)
            dt = t - self.t2
            return (self.H - self.D) + (self.v_max * dt) - (0.5 * self.a_max * (dt ** 2))
        elif t < self.t4: # Phase 4: Traverse (Maintain max height)
            return self.H
        elif t < self.t5: # Phase 5: Corner 2 (Accelerate downwards)
            dt = t - self.t4
            return self.H - (0.5 * self.a_max * (dt ** 2))
        elif t < self.t6: # Phase 6: Place (Cruise downwards at max speed)
            dt = t - self.t5
            return (self.H - self.D) - (self.v_max * dt)
        elif t <= self.t7: # Phase 7: Place (Decelerate downwards to a stop)
            dt = t - self.t6
            return self.D - (self.v_max * dt) + (0.5 * self.a_max * (dt ** 2))
        else:
            return 0.0

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlannerNode()
    
    # --- MOVEMENT 1 (Pick and Place) ---
    movement = PICK_AND_PLACE_POINTS['MOVEMENT_1']
    node.send_trajectory_goal(movement['P1'], movement['P4'], movement['height'])

    # The action is asynchronous, meaning the script will keep running.
    # We use this while loop to manually block the script and force it to wait 
    # until `self.is_action_done` flips to True.
    while rclpy.ok() and not node.is_action_done:
        rclpy.spin_once(node)
        
    # Check if the first movement succeeded before attempting the return trip
    if node.action_result and node.action_result.success: 
        node.get_logger().info('Pick completed successfully! Now returning home...')
        
        # --- MOVEMENT 2 (Return Trip) ---
        movement = PICK_AND_PLACE_POINTS['MOVEMENT_2']
        # Set grasp=False so it keeps the gripper open, leaving the object behind!
        node.send_trajectory_goal(movement['P1'], movement['P4'], movement['height'], grasp=False)
        
        # Block and wait for Movement 2 to finish
        while rclpy.ok() and not node.is_action_done:
            rclpy.spin_once(node)
            
        node.get_logger().info('All movements complete!')
    else: 
        node.get_logger().error('Pick failed. Cannot proceed to return trip.')
       
    # Clean shutdown of the ROS 2 node
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()