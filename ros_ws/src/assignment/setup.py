from setuptools import find_packages, setup

package_name = 'assignment'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/trajectories.json',
                                               'config/vel_trajectories.json',
                                               'config/pick_place_config.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andre',
    maintainer_email='andre@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'set_joint_conf       = assignment.joint_state_setter:main',
            'read_ee_pose         = assignment.read_ee_position:main',
            'workspace_visualizer = assignment.workspace_visualizer:main',
            'trajectory_follower      = assignment.trajectory_follower:main',
            'vel_trajectory_follower  = assignment.vel_trajectory_follower:main',
            'pick_place_impedance     = assignment.pick_place_impedance:main',
        ],
    },
)
