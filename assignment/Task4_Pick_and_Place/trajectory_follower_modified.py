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
    
    def _publish_position(self, q_list, gripper_pos):
        msg = JointTrajectory()
        msg.joint_names = self.robot.joint_names
        point = JointTrajectoryPoint()
        # In Position Control, we send 'positions' instead of 'velocities'
        point.positions = [float(v) for v in q_list] + [float(gripper_pos)]
        point.time_from_start.nanosec = int(1e9 / 10) # Timing for the controller
        msg.points = [point]
        self._joint_cmd_pub.publish(msg)


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
        raw_traj = self.generate_trajectory()
    
        # Check if the first element is a list (meaning we have segments)
        if len(raw_traj) > 0 and isinstance(raw_traj[0], list):
            # Return a list of lists of poses
            return [[point['target_pose'] for point in segment] for segment in raw_traj]
        else:
            # Return a single list of poses (original behavior)
            return [point['target_pose'] for point in raw_traj]


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
    def __init__(self, p1, p2, p3, center, normal_axis='z', num_points=20):
        """
        p1: First extreme point (local 2D)
        p2: Upper peak point (local 2D)
        p3: Second extreme point (local 2D)
        segment: Choice of trajectory (1, 2, 3, or 4)
        """
        super().__init__(center=center, normal_axis=normal_axis, num_points=num_points)
        self.p1 = np.array(p1)
        self.p2 = np.array(p2)
        self.p3 = np.array(p3)

        # The local 2D center is the midpoint between the two extremes
        self.local_center = (self.p1 + self.p3) / 2.0
        
    def generate_trajectory(self):
        traj1, traj2, traj3, traj4 = [], [], [], []
        
        # Vectors defining the ellipse axes in 2D
        u = self.p3 - self.local_center  # Vector corresponding to t = 0
        v = self.p2 - self.local_center  # Vector corresponding to t = pi/2
        
        # Helper function to generate a segment based on a range of t
        def create_segment(start_t, end_t):
            segment_list = []
            t_values = np.linspace(start_t, end_t, self.num_points)
            for t in t_values:
                point_2d = self.local_center + u * np.cos(t) + v * np.sin(t)
                target_pose = self.map_2d_to_3d(point_2d[0], point_2d[1])
                segment_list.append({'target_pose': target_pose, 'parameter': t})
            return segment_list

        # Forward Trajectories
        segments = [
            create_segment(np.pi, np.pi/2), # 0: P1 -> P2
            create_segment(np.pi/2, 0),     # 1: P2 -> P3
            create_segment(0, np.pi/2),     # 2: P3 -> P2
            create_segment(np.pi/2, np.pi)  # 3: P2 -> P1
        ]
        # Return ONLY the requested segment so get_poses() works
        return segments

class LinearTrajectory3D(ShapeGenerator):
    def __init__(self, p1, p2, num_points=20):
        """
        p1: Start point (3D)
        p2: End point (3D)
        """
        super().__init__(center=None, normal_axis=None, num_points=num_points)
        self.p1 = np.array(p1)
        self.p2 = np.array(p2)

    def generate_trajectory(self):
        trajectory = []

        t_values = np.linspace(0, 1, self.num_points)

        for t in t_values:
            # Linear interpolation in 3D
            point_3d = (1 - t) * self.p1 + t * self.p2

            # Match HalfEllipseTrajectory format EXACTLY
            target_pose = [
                float(point_3d[0]),
                float(point_3d[1]),
                float(point_3d[2]),
                float(np.pi),
                0.0
            ]

            trajectory.append({
                'target_pose': target_pose,
                'parameter': t
            })

        return trajectory


def main():
    rclpy.init()
    
    executor = TrajectoryExecutor()

    all_poses = []

    half_ellipse = HalfEllipseTrajectory(
        p1=[-0.2, 0.0],
        p2=[0.0, 0.25],
        p3=[0.2, 0.0],
        center=(0.0, 0.0, 0.1),
        normal_axis='z',
        num_points=20
    )
    all_poses.append(half_ellipse.get_poses())

    line = LinearTrajectory3D(
        p1=[-0.2, 0.0, 0.1],
        p2=[-0.2, 0.0, 0.0],
        num_points=5
    )
    all_poses.append(line.get_poses()) #4

    line = LinearTrajectory3D(
        p1=[-0.2, 0.0, 0.0],
        p2=[-0.2, 0.0, 0.1],
        num_points=5
    )
    all_poses.append(line.get_poses()) #5

    line = LinearTrajectory3D(
        p1=[0.2, 0.0, 0.1],
        p2=[0.2, 0.0, 0.0],
        num_points=5
    )
    all_poses.append(line.get_poses()) #6

    line = LinearTrajectory3D(
        p1=[0.2, 0.0, 0.0],
        p2=[0.2, 0.0, 0.1],
        num_points=5
    )
    all_poses.append(line.get_poses()) #7

    # To position
    executor.execute_trajectories(all_poses[3], trajectory_time=7)
    # Down to pick
    #executor.execute_trajectories(all_poses[4], trajectory_time=7)
    # Pick
    executor._publish_position(np.zeros(5), 0.6)
    # Lift
    # executor.execute_trajectories(all_poses[5], trajectory_time=4)
    # To Place
    executor.execute_trajectories(all_poses[0], trajectory_time=7)
    executor.execute_trajectories(all_poses[1], trajectory_time=7)
    # Down to place
    # executor.execute_trajectories(all_poses[6], trajectory_time=4)
    # Place
    executor._publish_position(np.zeros(5), 1.57)
    # Lift
    # executor.execute_trajectories(all_poses[7], trajectory_time=7)
    # Home
    executor.execute_trajectories(all_poses[2], trajectory_time=7)
    
    try:
        rclpy.spin(executor)
    except KeyboardInterrupt:
        pass
    finally:
        executor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()