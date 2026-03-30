from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'assignment'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py')) + glob(os.path.join('launch', 'sim_pick_and_place.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tylacoid',
    maintainer_email='albert.torrente.marti@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'visualize_ee_trajectory = assignment.visualize_ee_trajectory:main',
            'simple_trajectory_follower = assignment.simple_trajectory_follower:main',
            'vel_trajectory_controller  = assignment.vel_trajectory_controller:main',
            'set_joint_conf = assignment.set_joint_conf:main',
            'read_ee_cart = assignment.read_ee_cart:main',
            'publish_ee_pose = assignment.publish_ee_pose:main',
            'LTPB_trajectory_action_client = assignment.LTPB_trajectory_action_client:main',
            'workspace_visualizer = assignment.workspace_visualizer:main',
            'trajectory_follower_action_server = assignment.trajectory_controller_action_server:main',
        ],
    },
)
