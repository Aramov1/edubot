import sys
import os
import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Adjust these paths as per your workspace
_ASSIGNMENT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_PROJECT_ROOT   = os.path.dirname(_ASSIGNMENT_DIR)
sys.path.insert(0, _ASSIGNMENT_DIR)
sys.path.insert(0, os.path.join(_PROJECT_ROOT, 'ros_ws', 'src', 'python_controllers'))

from robot_kinematics import RobotKinematics
from python_controllers.set_joint_position import JOINT_NAMES

class BaseTrajectoryFollower(Node):
    """Base class for trajectory generation and execution."""
    def __init__(self, node_name, center, normal_axis='z', num_points=50):
        super().__init__(node_name)
        self.robot = RobotKinematics()
        self._joint_cmd_pub = self.create_publisher(JointTrajectory, 'joint_cmds', 10)
        
        self.center = center
        self.normal_axis = normal_axis
        self.num_points = num_points
        
    def generate_trajectory(self):
        """Must be implemented by subclasses."""
        raise NotImplementedError("Subclasses must implement generate_trajectory()")
        
    def map_2d_to_3d(self, x_2d, y_2d):
        """Helper to map a 2D local shape onto the correct 3D plane based on normal_axis."""
        cx, cy, cz = self.center
        if self.normal_axis == 'z':
            return [cx + x_2d, cy + y_2d, cz]
        elif self.normal_axis == 'y':
            return [cx + x_2d, cy, cz + y_2d]
        elif self.normal_axis == 'x':
            return [cx, cy + x_2d, cz + y_2d]
        else:
            raise ValueError("normal_axis must be 'x', 'y', or 'z'")

    def trajectory_to_pose(self, trajectory):
        return [point['target_pose'] for point in trajectory]
    
    def joint_angles_calculation(self, poses):
        joint_trajectory = []
        for pose in poses:
            solutions = self.robot.inverse_kinematics(pose, n_restarts=30)
            if solutions:
                joint_trajectory.append(solutions[0])  # Take the first solution
            else:
                joint_trajectory.append(None)  # No solution found
        return joint_trajectory
    
    def execute_trajectory(self, joint_trajectory, trajectory_time=10.0):
        self.joint_angles_trajectory = [j for j in joint_trajectory if j is not None]
        
        if not self.joint_angles_trajectory:
            print("No valid joint configurations to execute!")
            return
        
        self.time_per_point = trajectory_time / len(self.joint_angles_trajectory)
        self.current_point_idx = 0
        
        print(f"Executing {len(self.joint_angles_trajectory)} points. Look at RViz!")
        self.timer = self.create_timer(self.time_per_point, self.timer_callback)

    def timer_callback(self):
        if self.current_point_idx >= len(self.joint_angles_trajectory):
            self.timer.cancel()
            print("Trajectory execution completed!")
            return
        
        joint_angles = self.joint_angles_trajectory[self.current_point_idx]
        joint_angles_with_gripper = np.append(joint_angles, 0.0)
        
        traj_msg = JointTrajectory()
        traj_msg.joint_names = JOINT_NAMES
        
        point = JointTrajectoryPoint()
        point.positions = list(joint_angles_with_gripper)
        point.time_from_start = rclpy.time.Duration(seconds=0.0).to_msg()
        
        traj_msg.points.append(point)
        self._joint_cmd_pub.publish(traj_msg)
        
        self.current_point_idx += 1


class CircleTrajectoryFollower(BaseTrajectoryFollower):
    def __init__(self, center, radius, normal_axis='z', num_points=50):
        super().__init__('circle_trajectory_follower', center, normal_axis, num_points)
        self.radius = radius
        
    def generate_trajectory(self):
        trajectory = []
        for i in range(self.num_points):
            t = 2 * np.pi * i / self.num_points
            x_2d = self.radius * np.cos(t)
            y_2d = self.radius * np.sin(t)
            
            target_pose = self.map_2d_to_3d(x_2d, y_2d)
            trajectory.append({'target_pose': target_pose, 'parameter': t})
        return trajectory


class SquareTrajectoryFollower(BaseTrajectoryFollower):
    def __init__(self, center, side_length, normal_axis='z', num_points=50):
        super().__init__('square_trajectory_follower', center, normal_axis, num_points)
        self.side_length = side_length
        
    def generate_trajectory(self):
        trajectory = []
        L = self.side_length / 2.0
        # Define the 4 corners of the square
        corners = [(-L, -L), (L, -L), (L, L), (-L, L), (-L, -L)]
        
        # Divide points evenly among the 4 edges
        points_per_edge = max(1, self.num_points // 4)
        
        for i in range(4):
            start_x, start_y = corners[i]
            end_x, end_y = corners[i+1]
            for j in range(points_per_edge):
                t = j / points_per_edge
                x_2d = start_x * (1 - t) + end_x * t
                y_2d = start_y * (1 - t) + end_y * t
                
                target_pose = self.map_2d_to_3d(x_2d, y_2d)
                trajectory.append({'target_pose': target_pose})
        return trajectory


class TriangleTrajectoryFollower(BaseTrajectoryFollower):
    def __init__(self, center, side_length, normal_axis='z', num_points=50):
        super().__init__('triangle_trajectory_follower', center, normal_axis, num_points)
        self.side_length = side_length
        
    def generate_trajectory(self):
        trajectory = []
        # Calculate parameters for an equilateral triangle
        R = (self.side_length * np.sqrt(3)) / 3.0  # Distance from center to vertex
        
        # Define the 3 corners (pointing "up" relative to its local 2D plane)
        corners = [
            (0, R), 
            (-self.side_length / 2.0, -R / 2.0), 
            (self.side_length / 2.0, -R / 2.0),
            (0, R)
        ]
        
        # Divide points evenly among the 3 edges
        points_per_edge = max(1, self.num_points // 3)
        
        for i in range(3):
            start_x, start_y = corners[i]
            end_x, end_y = corners[i+1]
            for j in range(points_per_edge):
                t = j / points_per_edge
                x_2d = start_x * (1 - t) + end_x * t
                y_2d = start_y * (1 - t) + end_y * t
                
                target_pose = self.map_2d_to_3d(x_2d, y_2d)
                trajectory.append({'target_pose': target_pose})
        return trajectory


def main():
    rclpy.init()
    
    # ---------------------------------------------------------
    # Example: Choose which trajectory you want to execute here
    # ---------------------------------------------------------
    
    #follower = CircleTrajectoryFollower(
    #   center=(0.2, 0.2, 0.2), 
    #   radius=0.1
    #)
    
    # follower = SquareTrajectoryFollower(
    #   center=(0.2, 0.2, 0.2), 
    #   side_length=0.15
    #)
    
    follower = TriangleTrajectoryFollower(
        center=(0.2, 0.2, 0.2), 
        side_length=0.15, 
        normal_axis='z', 
        num_points=60  # Using a multiple of 3 is best for triangles
    )
    
    trajectory = follower.generate_trajectory()
    poses = follower.trajectory_to_pose(trajectory)  
    joint_trajectory = follower.joint_angles_calculation(poses)
    
    # Execute the trajectory (will update robot position in RViz)
    follower.execute_trajectory(joint_trajectory, trajectory_time=15.0)
    
    try:
        rclpy.spin(follower)
    except KeyboardInterrupt:
        pass
    
    follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()