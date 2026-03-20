from setuptools import find_packages, setup

package_name = 'python_controllers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/trajectories.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anton',
    maintainer_email='a.bredenbeck@tudelft.nl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'example_pos_traj = python_controllers.example_pos_traj:main',
            'example_vel_traj = python_controllers.example_vel_traj:main',
            'set_joint_position = python_controllers.set_joint_position:main',
            'read_ee_pose = python_controllers.read_ee_pose:main',
            'read_EE_position = python_controllers.read_EE_position:main',
            'publish_ee_pose = python_controllers.publish_ee_pose:main',
            'visualize_ee_trajectory = python_controllers.visualize_ee_trajectory:main',
            'trajectory_follower = python_controllers.trajectory_follower:main',
            'workspace_visualizer = python_controllers.workspace_visualizer:main',
            'z_vel_controller = python_controllers.z_vel_controller:main',
        ],
    },
)
