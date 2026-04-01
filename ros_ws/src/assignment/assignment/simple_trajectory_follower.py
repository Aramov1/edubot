import sys
import os
_ASSIGNMENT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_PROJECT_ROOT   = os.path.dirname(_ASSIGNMENT_DIR)
sys.path.insert(0, _ASSIGNMENT_DIR)
sys.path.insert(0, os.path.join(_PROJECT_ROOT, 'ros_ws', 'src', 'assignmet'))

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data  # <-- ADDED THIS IMPORT
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from .robot_kinematics import RobotKinematics


class TrajectoryExecutor(Node):
    """Single ROS2 Node responsible for calculating IK and executing trajectories."""
    def __init__(self, node_name='trajectory_executor'):
        super().__init__(node_name)
        self.robot = RobotKinematics()
        self._joint_cmd_pub = self.create_publisher(JointTrajectory, 'joint_cmds', 10)
        
        # --- FIXED: Applied qos_profile_sensor_data to match the publisher's QoS ---
        self._joint_state_sub = self.create_subscription(
            JointState, 
            'joint_states', 
            self.joint_state_callback, 
            qos_profile_sensor_data
        )
        
        self.joint_angles_trajectory = []
        
        # Initialize state variables to prevent AttributeError
        self.current_q_arm = None
        self.current_q_gripper = None
        self.waiting_for_start = False

    def joint_state_callback(self, msg):
        """Keep track of current joint states for potential use in IK seed or FK fallback."""
        self.current_q_arm = np.array(msg.position[:5])
        if len(msg.position) > 5:
            self.current_q_gripper = msg.position[5]

        
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
            self.get_logger().warn("No valid joint configurations to execute!")
            return
        
        self.time_per_point = trajectory_time / len(self.joint_angles_trajectory)
        self.current_point_idx = 0
        
        # Trigger the waiting state
        self.waiting_for_start = True 
        
        self.get_logger().info(f"Executing {len(self.joint_angles_trajectory)} points sequentially. Look at RViz!")
        self.timer = self.create_timer(self.time_per_point, self.timer_callback)

    def timer_callback(self):
        if self.current_point_idx >= len(self.joint_angles_trajectory):
            self.timer.cancel()
            self.get_logger().info("All trajectory executions completed!")
            return
        
        # Start position synchronization logic
        if self.waiting_for_start:
            if self.current_q_arm is None:
                self.get_logger().warn("Waiting for joint_states data...", throttle_duration_sec=2.0)
                return
            
            target_start_joints = self.joint_angles_trajectory[0]
            
            # Calculate the Euclidean distance (error) between current state and target start state
            error = np.linalg.norm(self.current_q_arm - target_start_joints[:len(self.current_q_arm)])
            
            # 0.05 radians (~2.8 degrees) is usually a good tolerance, adjust if hardware is jittery
            if error > 0.05: 
                self.get_logger().info(f"Moving to start position... Current error: {error:.3f} rad", throttle_duration_sec=1.0)
                # Keep commanding the first point to ensure the robot gets there
                self._publish_point(target_start_joints)
                return  # Exit callback early, do not increment current_point_idx
            else:
                self.get_logger().info("Start position reached! Executing full trajectory...")
                self.waiting_for_start = False

        # Get current point and publish
        joint_angles = self.joint_angles_trajectory[self.current_point_idx]
        self._publish_point(joint_angles)
        
        self.current_point_idx += 1

    def _publish_point(self, joint_angles):
        """Helper to format and publish a single joint command."""
        joint_angles_with_gripper = np.append(joint_angles, 0.0)
        
        traj_msg = JointTrajectory()
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        traj_msg.joint_names = self.robot.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = list(joint_angles_with_gripper)
        point.time_from_start = rclpy.time.Duration(seconds=self.time_per_point).to_msg()
        
        traj_msg.points.append(point)
        self._joint_cmd_pub.publish(traj_msg)

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

    executor.get_logger().info("Generating trajectories...")
    
    circle = CircleTrajectory(
        center=(0, 0.25, 0), 
        radius=0.05,
        num_points=30
    )

    all_poses = []

    executor.get_logger().info("Computing IK for all trajectory points...")
    
    all_poses.extend(circle.get_poses())
    
    total_time = 10 * 1  # 10 seconds per shape, rough estimate
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