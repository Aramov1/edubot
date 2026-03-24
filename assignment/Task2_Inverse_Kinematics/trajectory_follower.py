import sys
import os
_ASSIGNMENT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_PROJECT_ROOT   = os.path.dirname(_ASSIGNMENT_DIR)
sys.path.insert(0, _ASSIGNMENT_DIR)
sys.path.insert(0, os.path.join(_PROJECT_ROOT, 'ros_ws', 'src', 'assignmet'))

import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from robot_kinematics import RobotKinematics


class TrajectoryExecutor(Node):
    """Single ROS2 Node responsible for calculating IK and executing trajectories."""
    def __init__(self, node_name='trajectory_executor'):
        super().__init__(node_name)
        self.robot = RobotKinematics()
        self._joint_cmd_pub = self.create_publisher(JointTrajectory, 'joint_cmds', 10)
        self.joint_angles_trajectory = []
        
    def joint_angles_calculation(self, poses):
        joint_trajectory = []
        for pose in poses:
            solutions = self.robot.inverse_kinematics(pose, n_restarts=30)
            if solutions:
                joint_trajectory.append(solutions[0])  # Take the first solution
            else:
                joint_trajectory.append(None)  # No solution found
        return joint_trajectory
    
    def execute_trajectories(self, poses_list, trajectory_time=10.0):
        """Executes a continuous list of poses."""
        joint_trajectory = self.joint_angles_calculation(poses_list)
        self.joint_angles_trajectory = [j for j in joint_trajectory if j is not None]
        
        if not self.joint_angles_trajectory:
            print("No valid joint configurations to execute!")
            return
        
        self.time_per_point = trajectory_time / len(self.joint_angles_trajectory)
        self.current_point_idx = 0
        
        print(f"Executing {len(self.joint_angles_trajectory)} points sequentially. Look at RViz!")
        self.timer = self.create_timer(self.time_per_point, self.timer_callback)

    def timer_callback(self):
        if self.current_point_idx >= len(self.joint_angles_trajectory):
            self.timer.cancel()
            print("All trajectory executions completed!")
            return
        
        joint_angles = self.joint_angles_trajectory[self.current_point_idx]
        joint_angles_with_gripper = np.append(joint_angles, 0.0)
        
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.robot.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = list(joint_angles_with_gripper)
        point.time_from_start = rclpy.time.Duration(seconds=0.0).to_msg()
        
        traj_msg.points.append(point)
        self._joint_cmd_pub.publish(traj_msg)
        
        self.current_point_idx += 1


# ==========================================
# SHAPE GENERATORS (Pure Math, No ROS Nodes)
# ==========================================

class ShapeGenerator:
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
        return [point['target_pose'] for point in self.generate_trajectory()]


class CircleTrajectory(ShapeGenerator):
    def __init__(self, center, radius, normal_axis='z', num_points=50):
        super().__init__(center, normal_axis, num_points)
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


class SquareTrajectory(ShapeGenerator):
    def __init__(self, center, side_length, normal_axis='z', num_points=50):
        super().__init__(center, normal_axis, num_points)
        self.side_length = side_length
        
    def generate_trajectory(self):
        trajectory = []
        L = self.side_length / 2.0
        corners = [(-L, -L), (L, -L), (L, L), (-L, L), (-L, -L)]
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


class TriangleTrajectory(ShapeGenerator):
    def __init__(self, center, side_length, normal_axis='z', num_points=50):
        super().__init__(center, normal_axis, num_points)
        self.side_length = side_length
        
    def generate_trajectory(self):
        trajectory = []
        R = (self.side_length * np.sqrt(3)) / 3.0 
        corners = [
            (0, R), 
            (-self.side_length / 2.0, -R / 2.0), 
            (self.side_length / 2.0, -R / 2.0),
            (0, R)
        ]
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


def main():
    rclpy.init()
    
    executor = TrajectoryExecutor()
    
    circle = CircleTrajectory(
        center=(0.15, 0.15, 0), 
        radius=0.1,
        num_points=30
    )
    
    square = SquareTrajectory(
        center=(0.15, 0.15, 0.05), 
        side_length=0.1,
        num_points=40
    )
    
    half_ellipse = HalfEllipseTrajectory(
        p1=[-0.2, 0.0],
        p2=[0.0, 0.25],
        p3=[0.2, 0.0],
        segment=2,
        normal_axis='z',
        num_points=60,
        center=(0.0, 0.0, 0.1),
    )
    
    all_poses = []
    
    all_poses.extend(circle.get_poses())
    all_poses.extend(square.get_poses())
    all_poses.extend(half_ellipse.get_poses())
    
    total_time = 15.0 * 3  # 15 seconds per shape, rough estimate
    executor.execute_trajectories(all_poses, trajectory_time=total_time)
    
    try:
        rclpy.spin(executor)
    except KeyboardInterrupt:
        pass
    finally:
        executor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()