from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    lerobot_pkg_dir = get_package_share_directory('lerobot')
    assignment_pkg_dir = get_package_share_directory('assignment')
    
    # Path to sim_position.launch.py from lerobot
    hw_velocity_launch_file = os.path.join(
        lerobot_pkg_dir, 
        'launch', 
        'hw_velocity.launch.py'
    )
    
    # Include the sim_position launch file
    hw_velocity_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hw_velocity_launch_file)
    )
    
    # Node 1: Publish End-Effector Pose
    publish_ee_pose_node = Node(
        package='assignment',
        executable='publish_ee_pose',
        name='publish_ee_pose',
        output='screen'
    )
    
    # Node 3: Simple Trajectory Follower
    vel_trajectory_follower_node = Node(
        package='assignment',
        executable='vel_trajectory_follower',
        name='vel_trajectory_follower',
        output='screen'
    )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add all actions in order
    ld.add_action(hw_velocity_launch)
    ld.add_action(publish_ee_pose_node)
    ld.add_action(vel_trajectory_follower_node)
    
    return ld
