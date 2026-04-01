from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    lerobot_pkg_dir = get_package_share_directory('lerobot')

    sim_velocity_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lerobot_pkg_dir, 'launch', 'sim_velocity.launch.py')
        )
    )

    publish_ee_pose_node = Node(
        package='assignment',
        executable='publish_ee_pose',
        name='publish_ee_pose',
        output='screen'
    )

    competition_server_node = Node(
        package='assignment',
        executable='competition_server',
        name='competition_server',
        output='screen'
    )

    competition_client_node = Node(
        package='assignment',
        executable='competition_client',
        name='competition_client',
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(sim_velocity_launch)
    ld.add_action(publish_ee_pose_node)
    ld.add_action(competition_server_node)
    ld.add_action(competition_client_node)

    return ld
