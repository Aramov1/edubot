
# pick_place_impedance_Andre
import os
import yaml
import time
import numpy as np

# Used to create States for State Machiene
from enum import Enum, auto 

# ROS Related packages
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ament_index_python.packages import get_package_share_directory

# Robot Kinematics File
from assignment.robot_kinematics import RobotKinematics

robot = RobotKinematics()

# Configuration File
DEFAULT_CONFIG_FILE = "pick_place_config.yml"

class ShapeGenerator():
    """Base class for math trajectory generation."""
    def __init__(self, center, normal_axis='z', num_points=50):
        self.center = center
        self.normal_axis = normal_axis
        self.num_points = num_points
        
    def generate_trajectory(self):
        """Must be implemented by subclasses."""
        raise NotImplementedError("Subclasses must implement generate_trajectory()")
        
    def map_2d_to_3d(self, x_2d, y_2d):
        """Helper to map a 2D local shape onto the correct 3D plane."""
        cx, cy, cz = self.center
        if self.normal_axis == 'z':
            return [cx + x_2d, cy + y_2d, cz, np.pi, 0.0]
        elif self.normal_axis == 'y':
            return [cx + x_2d, cy, cz + y_2d, np.pi, 0.0]
        elif self.normal_axis == 'x':
            return [cx, cy + x_2d, cz + y_2d, np.pi, 0.0]
        else:
            raise ValueError("normal_axis must be 'x', 'y', or 'z'")

    def get_poses(self):
        """Extracts just the target poses from the trajectory dictionary."""
        raw_traj = self.generate_trajectory()
    
        # Check if the first element is a list (meaning we have segments)
        if len(raw_traj) > 0 and isinstance(raw_traj[0], list):
            # Return a list of lists of poses
            return [[point['target_pose'] for point in segment] for segment in raw_traj]
        else:
            # Return a single list of poses (original behavior)
            return [point['target_pose'] for point in raw_traj]
        
class HalfEllipseTrajectory(ShapeGenerator):
    def __init__(self, p1, p2, p3, segment, center, normal_axis='z', num_points=20):
        super().__init__(center=center, normal_axis=normal_axis, num_points=num_points)
        self.p1 = np.array(p1)
        self.p2 = np.array(p2)
        self.p3 = np.array(p3)
        self.segment = segment
        self.local_center = (self.p1 + self.p3) / 2.0
        
    def generate_trajectory(self):
        traj1 = []
        traj2 = []
        u = self.p3 - self.local_center  
        v = self.p2 - self.local_center 
        
        t1_values = np.linspace(np.pi, np.pi/2, self.num_points)
        for t in t1_values:
            point_2d = self.local_center + u * np.cos(t) + v * np.sin(t)
            target_pose = self.map_2d_to_3d(point_2d[0], point_2d[1])
            traj1.append({'target_pose': target_pose, 'parameter': t})
            
        t2_values = np.linspace(np.pi/2, 0, self.num_points)
        for t in t2_values:
            point_2d = self.local_center + u * np.cos(t) + v * np.sin(t)
            target_pose = self.map_2d_to_3d(point_2d[0], point_2d[1])
            traj2.append({'target_pose': target_pose, 'parameter': t})
            
        if self.segment == 1:
            return traj1
        elif self.segment == 2:
            return traj2
        else:
            raise ValueError("Segment must be 1 (p1 to p2) or 2 (p2 to p3)")

# State Machiene
class StateMachine(Enum):
    HOME     = auto()
    MOVE     = auto()
    MOVE_PI     = auto()
    MOVE_PL     = auto()
    DESCEND  = auto()
    LIFT     = auto()
    GRASP    = auto()
    RELEASE  = auto()
    SWAP     = auto()
    DONE     = auto()

class MotionControllerPos():
    """Handles the Trajectory (Linear/Elipse) between two Targets, IK selection of the closest solution, 
    and index tracking of the target moving points."""
    def __init__(self, n_points: int = 20):
        self.robot = RobotKinematics()
        
        # Creation
        self.n_points    = n_points # Number of points to generate between two targets
        self.path_buffer = []       #  List of joint configurations [q1, q2, ... qn]
        self.current_idx = 0
        
    def generate_ellipse_path(self, start_xyz, end_xyz, height_offset, current_q):
        """Generates a 2-segment half-ellipse and converts to IK solutions."""
        
        # Define the peak of the ellipse (p2)
        mid_z = max(start_xyz[2], end_xyz[2]) + height_offset
        p2 = [(start_xyz[0] + end_xyz[0])/2, (start_xyz[1] + end_xyz[1])/2, mid_z]
        
        # Use existing HalfEllipseTrajectory segments
        # n_points points for up-arc, n_points points for down-arc
        seg1 = HalfEllipseTrajectory(start_xyz, p2, end_xyz, segment=1, center=[0,0,0], num_points=10).get_poses()
        seg2 = HalfEllipseTrajectory(start_xyz, p2, end_xyz, segment=2, center=[0,0,0], num_points=10).get_poses()
        
        full_poses = seg1 + seg2
        
        # 3. Convert Cartesian poses to best IK configurations
        last_q = current_q[:5]
        for pose in full_poses:
            q_best = self._get_best_ik(pose, last_q)
            if q_best:
                self.path_buffer.append(q_best)
                last_q = q_best # Update reference to ensure continuity
                
    def generate_linear_point(self, target_xyz, current_q):
        """Generates a single-point 'path' for DESCEND or LIFT_SAFE."""
        self.path_buffer = []
        self.current_idx = 0
        
        target_pose = [*target_xyz, np.pi, 0.0] # Gripper down orientation
        q_best = self._get_best_ik(target_pose, current_q[:5])
        if q_best:
            self.path_buffer.append(q_best)

    def _get_best_ik(self, target_pose, reference_q):
        """Pick the IK solution closest to the reference (previous) configuration."""
        solutions = self.robot.inverse_kinematics(target_pose)
        if not solutions:
            return None
        # Minimize joint displacement to prevent sudden flips
        distances = [np.linalg.norm(np.array(sol) - reference_q) for sol in solutions]
        return solutions[np.argmin(distances)]

    def get_next_command(self):
        """Returns the next q-set in the path. Returns None if path is finished."""
        if self.current_idx < len(self.path_buffer):
            cmd = self.path_buffer[self.current_idx]
            self.current_idx += 1
            return cmd
        return None

    def is_finished(self):
        return self.current_idx >= len(self.path_buffer)

class MotionControllerVel(MotionControllerPos):
    """ Cartesian Velocity Controller"""
    def __init__(self, cfg_file):
        super().__init__()
        motion_ctr_cfg = cfg_file['motion_ctr']
        
        # Diagonal matrices for Stiffness (K) and Damping (D)
        self.K = np.diag([motion_ctr_cfg['k_xy'],  motion_ctr_cfg['k_xy'],  motion_ctr_cfg['k_z'], 
                          motion_ctr_cfg['k_rot'], motion_ctr_cfg['k_rot'], motion_ctr_cfg['k_rot']])
        self.D = np.diag([motion_ctr_cfg['d_xy'],  motion_ctr_cfg['d_xy'],  motion_ctr_cfg['d_z'], 
                          motion_ctr_cfg['d_rot'], motion_ctr_cfg['d_rot'], motion_ctr_cfg['d_rot']])
        
        # self.max_v = motion_ctr_cfg['max_cart_vel_m_s']

    def compute(self, x_des, x_cur, dx_cur, J_inv, max_dq):
        # 1. Computer Error
        e_pos = x_des - x_cur
        e_pos[3:] = (e_pos[3:] + np.pi) % (2 * np.pi) - np.pi # Wrap angles
        
        e_vel = -dx_cur   

        # 2. Control Law: v = K*error - D*velocity
        v_cart = self.K @ e_pos + self.D @ e_vel
        # v_cart = np.clip(v_cart, -self.max_v, self.max_v)

        # 3.Compute Joint Velocity: dq = J^-1 * v
        dq = J_inv @ v_cart
        dq = np.clip(dq, -max_dq, max_dq)

        return dq

        
class MotionController_HEURISTIC_VEL(MotionControllerPos):
    def __init__(self, v_avg=0.1):
        super().__init__()
        self.v_avg = v_avg
        # Orientation limits: rotX in [-pi/2, 2]
        self.limit_min = -np.pi / 2
        self.limit_max = 2.0

    def get_velocity_command(self, current_q):
        if self.is_finished():
            return np.zeros(6), True

        # 1. Get current EE and target
        x_cur = np.array(self.robot.forward_kinematics(current_q[:5])).flatten()
        target_pose = self.path_buffer[self.current_idx]
        
        # 2. Translation components (Base Frame)
        dx = target_pose[0] - x_cur[0]
        dy = target_pose[1] - x_cur[1]
        dz = target_pose[2] - x_cur[2]
        dist = np.sqrt(dx**2 + dy**2 + dz**2)

        if dist < 0.005:
            self.current_idx += 1
            return self.get_velocity_command(current_q)

        # 3. Direct Joint Assignment (The "Non-Jacobian" way)
        dq = np.zeros(6)

        # q1 (Shoulder Rotation) handles the "Angular Error" in the XY plane
        # It rotates the arm to face the target.
        target_angle = np.arctan2(target_pose[1], target_pose[0])
        current_angle = current_q[0]
        dq[0] = 1.5 * (target_angle - current_angle) # Proportional heading control

        # q2, q3 (Shoulder/Elbow Pitch) handle Reach and Height
        # Instead of J-inv, we use the distance error to drive them
        # We scale the overall speed to match your v_avg
        speed_scale = self.v_avg / dist if dist > 0 else 0
        
        # Simple heuristic: pitch joints react to Z and Radial distance errors
        radial_dist_err = np.sqrt(target_pose[0]**2 + target_pose[1]**2) - \
                          np.sqrt(x_cur[0]**2 + x_cur[1]**2)
        
        dq[1] = (radial_dist_err * 2.0 + dz * 1.0) * speed_scale
        dq[2] = (dz * 2.0 - radial_dist_err * 1.0) * speed_scale

        # 4. Orientation Safety (Wrist Pitch/Roll)
        # Manually clamp rotX and rotY by adjusting q4 and q5
        dq[3] = 1.0 * (target_pose[3] - x_cur[3]) # Correct rotX
        dq[4] = 1.0 * (target_pose[4] - x_cur[4]) # Correct rotY

        # 5. Final Safety: Enforce your [-pi/2, 2] bounds
        # If the next step would exceed limits, zero the velocity
        next_rot_x = x_cur[3] + (dq[3] * 0.05) # Predict next state (20Hz)
        if next_rot_x < self.limit_min or next_rot_x > self.limit_max:
            dq[3] = 0.0
            
        return np.clip(dq, -1.5, 1.5), False
    


class PickPlaceNode(Node):
    def __init__(self, hz = 10):
        super().__init__('pick_place_node')
        self.get_logger().info("Starting PickPlaceNode...")

        # Configuration & Kinematics
        self.hz = hz
        self.cfg   = self._load_config_file(DEFAULT_CONFIG_FILE)
        self.robot = RobotKinematics()
        #self.ctrl  = MotionController(self.cfg)
        #self.ctrl  = MotionControllerPosition(n_points=20)
        self.ctrl   = MotionControllerPos(v_avg=0.15)


        # State & Movement Variables
        self.state = StateMachine.HOME
        self.current_q = None
        self.prev_x = None
        self.dx_filtered = np.zeros(6)
        self.cycle_count = 0

        # Extract Locations
        self.pick_queue  = [np.array([p['x'], p['y'], p['z']]) for p in self.cfg['pick_points']]
        self.place_queue = [np.array([p['x'], p['y'], p['z']]) for p in self.cfg['place_points']]
        
        # Targets for the active cycle updated in SWAP state
        self.point_index = -1
        self.cycles = self.cfg['motion']['cycles']
        
        # Define home Joint Configuration
        self.home_q = np.array(self.cfg['home']['joint_angles_rad'])

        # Initialize gripper to open position
        self.gripper_val = self.cfg['gripper_ctr']['open_angle_rad']
        
        # ROS Setup
        self.pub = self.create_publisher(JointTrajectory, 'joint_cmds', 10)
        self.sub = self.create_subscription(JointState, 'joint_states', self._on_state, qos_profile_sensor_data)
        self.timer = self.create_timer(1/self.hz, self._control_loop_position) 

        self.get_logger().info("PickPlaceNode started successfully!")

    def _load_config_file(self, config_file: str):
        # Locate the file in the 'install' directory
        config_path = os.path.join(
            get_package_share_directory('assignment'), 
            'config', 
            config_file
        )
        
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)
        
    def _on_state(self, msg):
        # Map joint names to ordered array
        mapping = dict(zip(msg.name, msg.position))
        self.current_q = np.array([mapping.get(name, 0.0) for name in self.robot.joint_names])

    # def _control_loop(self):
    #     "Implementation of Pick & Place State Space Machiene Logic"
    #     if self.current_q is None: 
    #         self.get_logger().warn("Waiting For Joint States...")
    #         time.sleep(1)
    #         return
    #     self.get_logger().info(f"Current state: {self.state}")
    #     # Get Current Pose (FK) and Velocity (Finite Difference)
    #     x_cur  = self.robot.forward_kinematics(self.current_q[:5])
    #     dx_cur = self._get_velocity(x_cur)

    #     # Logic Switch
    #     if self.state == StateMachine.HOME:
    #         self._move_to_pose(self._get_home_pose(), x_cur, dx_cur)
    #         if self._at_target(self._get_home_pose()[:3], x_cur[:3], 'thresh_soft'):
    #             self.state = StateMachine.DESCEND

    #     elif self.state == StateMachine.DESCEND:
    #         self._move_to_pose(self._combine(self.pick_pos), x_cur, dx_cur)
    #         if self._at_target(self.pick_pos, x_cur[:3], 'thresh_soft'):
    #             self.state = StateMachine.GRASP
    #             self.start_time = time.time()

    #     elif self.state == StateMachine.GRASP:
    #         # Close gripper, hold arm still
    #         self._publish_velocity(np.zeros(5), self.cfg['gripper_ctr']['vel_rad_s'])
    #         if time.time() - self.start_time > self.cfg['motion']['dwell_grasp_s']:
    #             self.state = StateMachine.LIFT

    #     elif self.state == StateMachine.LIFT:
    #         target = np.array([self.pick_pos[0], self.pick_pos[1], self.cfg['motion']['approach_z']])
    #         self._move_to_pose(self._combine(target), x_cur, dx_cur)
    #         if self._at_target(target, x_cur[:3], 'thresh_soft'):
    #             self.state = StateMachine.MOVE

    #     elif self.state == StateMachine.MOVE:
    #         target = np.array([self.place_pos[0], self.place_pos[1], self.cfg['motion']['approach_z']])
    #         self._move_to_pose(self._combine(target), x_cur, dx_cur)
    #         if self._at_target(target, x_cur[:3], 'thresh_soft'):
    #             self.state = StateMachine.RELEASE
    #             self.start_time = time.time()

    #     elif self.state == StateMachine.RELEASE:
    #         # Open gripper
    #         self._publish_velocity(np.zeros(5), -self.cfg['gripper_ctr']['vel_rad_s'])
    #         if time.time() - self.start_time > self.cfg['motion']['dwell_release_s']:
    #             self.state = StateMachine.SWAP

    #     elif self.state == StateMachine.SWAP:
    #         if self.point_index < len(self.pick_queue):
    #             self.get_logger().info(f"Moving to object set #{self.point_index + 1}")
                
    #             self.pick_pos  = self.pick_queue[self.point_index]
    #             self.place_pos = self.place_queue[self.point_index]
                
    #             self.state = StateMachine.MOVE(self.pick_pos, self.place_pos)
    #         else:
    #             self.cycles -= 1 # One Cycle Completer
    #             if self.cycles:
    #                 # Revert Pick & Place Locations
    #                 self.pick_queue, self.place_queue = self.place_queue, self.pick_queue
    #                 self.point_index = -1
    #             else:
    #                 self.get_logger().info("All points completed!")
    #                 self.state = StateMachine.DONE

    def _control_loop_position(self):
        "DESIRED MOVEMENT OF THE ROBOT:"
        "1. SWAP -> HOME"
        "2. HOME -> MOVE_PI (to Pick + offset)"
        "3. MOVE_PI -> DESCEND (Pick position)"
        "4. DESCEND -> GRASP"
        "5. GRASP -> LIFT (Pick position)"
        "6. LIFT -> MOVE_PL (to Place + offset)"
        "7. MOVE_PL -> DESCEND (Place position)"
        "8. DESCEND -> RELEASE"
        "9. RELEASE -> LIFT (Place position)"
        "10. LIFT -> SWAP"
        "--- RESET FROM 1. ---"

        if self.current_q is None: return

        # 1. Check if the controller is currently executing a path
        if not self.ctrl.is_finished():
            target_q = self.ctrl.get_next_command()
            self._publish_position(target_q, self.gripper_val)
            return # Exit loop to wait for next timer tick

        # 2. If path is finished, handle State Transitions
        x_cur = self.robot.forward_kinematics(self.current_q[:5])[:3]

        if self.state == StateMachine.SWAP:
            if self.point_index < len(self.pick_queue):
                self.get_logger().info(f"Moving to object set #{self.point_index + 1}")
                
                self.pick_pos  = self.pick_queue[self.point_index]
                self.place_pos = self.place_queue[self.point_index]
                
                self.state = StateMachine.HOME
            else:
                self.cycles -= 1 # One Cycle Completer
                if self.cycles:
                    # Revert Pick & Place Locations
                    self.pick_queue, self.place_queue = self.place_queue, self.pick_queue
                    self.point_index = -1
                else:
                    self.get_logger().info("All points completed!")
                    self.state = StateMachine.DONE

        elif self.state == StateMachine.HOME:
            # Move directly to Home configuration
            self.ctrl.path_buffer = [self.home_q]
            self.ctrl.current_idx = 0
            self.state = StateMachine.MOVE_PI

        elif self.state == StateMachine.MOVE_PI:
            # Generate arc to 'safe' height above pick point
            safe_offset = self.cfg['motion']['approach_z'] + self.pick_pos[2]
            safe_pick = [self.pick_pos[0], self.pick_pos[1], safe_offset]
            self._publish_position(self.current_q, 0.5)         # Make sure the gripper is open for PICK
            self.ctrl.generate_ellipse_path(self, x_cur, safe_pick, 0.05, self.current_q)
            self.state = StateMachine.DESCEND

        elif self.state == StateMachine.MOVE_PL:
            # Generate arc to 'safe' height above place point
            safe_offset = self.cfg['motion']['approach_z'] + self.place_pos[2]
            safe_place = [self.place_pos[0], self.place_pos[1], safe_offset]
            self.ctrl.generate_ellipse_path(self, x_cur, safe_place, 0.05, self.current_q)
            self.state = StateMachine.DESCEND

        elif self.state == StateMachine.GRASP:
            self.gripper_val = self.cfg['gripper_ctr']['close_angle_rad']
            if time.time() - self.start_time > 1.0:
                self.state = StateMachine.LIFT

        elif self.state == StateMachine.LIFT:
            # Generate the vertical offset from pick/place position and move
            safe_offset = self.cfg['motion']['approach_z'] + self.place_pos[2]
            safe_place = [self.place_pos[0], self.place_pos[1], safe_offset]
            self.ctrl.generate_linear_point(safe_place, self.current_q)
            
            # Move to next state machine
            joint_pose = JointState()
            if joint_pose[4] <= 0.61:
                self.state = StateMachine.MOVE_PL
                self.start_time = time.time()
            elif joint_pose[4] > 0.61:            
                self.state = StateMachine.SWAP
                self.start_time = time.time()

        elif self.state == StateMachine.DESCEND:
            # Vertical descent to pick/place position
            self.ctrl.generate_linear_point(self.place_pos, self.current_q)

            # Move to next state machine
            joint_pose = JointState()
            if joint_pose[4] <= 0.61:
                self.state = StateMachine.RELEASE
                self.start_time = time.time()
            elif joint_pose[4] > 0.61:            
                self.state = StateMachine.GRASP
                self.start_time = time.time()

        elif self.state == StateMachine.RELEASE:
            self.gripper_val = self.cfg['gripper_ctr']['open_angle_rad']
            if time.time() - self.start_time > 1.0:
                self.state = StateMachine.LIFT

    def _control_loop_heuristic(self):
        """
        Main 25Hz Control Loop.
        1. Asks the MotionController for the velocity (dq) to reach the next waypoint.
        2. If a path is in progress, it publishes velocities.
        3. If a path is finished, it transitions to the next high-level state.
        """
        if self.current_q is None:
            self.get_logger().warn("Waiting for joint states...", throttle_duration_sec=2.0)
            return

        # --- 1. GET VELOCITY FROM HEURISTIC CONTROLLER ---
        # This call handles the internal path_buffer indexing and heuristic dq mapping.
        dq, path_finished = self.ctrl.get_velocity_command(self.current_q)

        # --- 2. EXECUTION PHASE ---
        if not path_finished:
            # We are currently traversing an ellipse or linear segment.
            # Publish the joint velocities calculated by the heuristic.
            self._publish_velocity(dq, self.gripper_val)
            return  # Exit early to wait for the next 25Hz tick

        # --- 3. STATE TRANSITION PHASE (Only runs if path_finished is True) ---
        # Get current EE position for the next trajectory generation
        x_cur = np.array(self.robot.forward_kinematics(self.current_q[:5])).flatten()

        if self.state == StateMachine.SWAP:
            self.point_index += 1
            if self.point_index < len(self.pick_queue):
                self.pick_pos = self.pick_queue[self.point_index]
                self.place_pos = self.place_queue[self.point_index]
                self.get_logger().info(f"Targeting Object {self.point_index + 1}")
                self.state = StateMachine.HOME
            else:
                self.state = StateMachine.DONE

        elif self.state == StateMachine.HOME:
            # Direct joint-space move to home
            self.ctrl.path_buffer = [self.home_q]
            self.ctrl.current_idx = 0
            self.state = StateMachine.APPROACH

        elif self.state == StateMachine.APPROACH:
            # Generate Ellipse: Current -> Above Pick Point
            z_safe = self.pick_pos[2] + self.cfg['motion']['approach_z']
            target_xyz = [self.pick_pos[0], self.pick_pos[1], z_safe]
            
            self.ctrl.generate_ellipse_path(x_cur[:3], target_xyz, 0.05, self.current_q)
            self.state = StateMachine.DESCEND

        elif self.state == StateMachine.DESCEND:
            # Linear drop to pick coordinates
            self.ctrl.generate_linear_point(self.pick_pos, self.current_q)
            self.state = StateMachine.GRASP
            self.start_time = time.time()

        elif self.state == StateMachine.GRASP:
            # Stop arm, close gripper
            self._publish_velocity(np.zeros(6), self.cfg['gripper_ctr']['vel_rad_s'])
            if time.time() - self.start_time > self.cfg['motion']['dwell_grasp_s']:
                self.state = StateMachine.LIFT

        elif self.state == StateMachine.LIFT:
            # Generate Ellipse: Current -> Above Place Point
            z_safe = self.place_pos[2] + self.cfg['motion']['approach_z']
            target_xyz = [self.place_pos[0], self.place_pos[1], z_safe]
            
            # Larger height offset (0.1) to clear obstacles while carrying object
            self.ctrl.generate_ellipse_path(self, x_cur[:3], target_xyz, 0.1, self.current_q)
            self.state = StateMachine.RELEASE # Or DESCEND_PLACE if you have that state
            self.start_time = time.time()

        elif self.state == StateMachine.RELEASE:
            # Stop arm, open gripper
            self._publish_velocity(np.zeros(6), -self.cfg['gripper_ctr']['vel_rad_s'])
            if time.time() - self.start_time > self.cfg['motion']['dwell_release_s']:
                self.state = StateMachine.SWAP

        elif self.state == StateMachine.DONE:
            self._publish_velocity(np.zeros(6), 0.0)
            self.get_logger().info("Task Complete.", throttle_duration_sec=10.0)

    def _get_velocity(self, x_cur):
        
        if self.prev_x is None: 
            self.prev_x = x_cur
            return np.zeros(6)
        
        # Compute Velocity using Finite Diferences
        raw_dx = (x_cur - self.prev_x) * self.hz

        # Low Pass Filter to reduce noise
        self.dx_filtered = 0.3 * raw_dx + 0.7 * self.dx_filtered
        self.prev_x = x_cur
        
        return self.dx_filtered
        
    def _move_to_pose(self, x_des, x_cur, dx):
        J_inv, _ = self.robot.jacobian_inverse(self.current_q[:5])
        dq = self.ctrl.compute(x_des, x_cur, dx, J_inv, self.cfg['motion_ctr']['max_joint_vel_rad_s'])
        self._publish_velocity(dq, 0.0)
        
    def _at_target(self, target, current, thresh_key):
        return np.linalg.norm(target - current) < self.cfg['motion'][thresh_key]

    def _get_home_pose(self):
        return self.robot.forward_kinematics(self.home_q)

    def _combine(self, xyz):
        # Maintain a downward orientation (Pi) while moving to target XYZ
        return np.array([*xyz, np.pi, 0.0, 0.0])

    def _publish_velocity(self, dq, gripper_v):
        # q_dot_safe = self._apply_joint_limit_clamp(q_dot)
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.robot.joint_names
        point = JointTrajectoryPoint()
        point.velocities = [float(v) for v in dq] + [float(gripper_v)]
        msg.points = [point]
        self.pub.publish(msg)

    def _publish_position(self, q_list, gripper_pos):
        msg = JointTrajectory()
        msg.joint_names = self.robot.joint_names
        point = JointTrajectoryPoint()
        # In Position Control, we send 'positions' instead of 'velocities'
        point.positions = [float(v) for v in q_list] + [float(gripper_pos)]
        point.time_from_start.nanosec = int(1e9 / self.hz) # Timing for the controller
        msg.points = [point]
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = PickPlaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


