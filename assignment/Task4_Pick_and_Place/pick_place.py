import sys
import os

import numpy as np
import sympy as sp
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ros_ws.build.python_controllers.build.lib.python_controllers.read_EE_position import EEPositionReader
from jacobian_33_modified import compute_velocities

# Initial position definition (resting position)
initial_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.8]
    # We retrieve the robot's current end-effector position using the EEPositionReader
current_position = EEPositionReader()


# DETERMINE THESE POSITIONS BETTER WITH THE ROBOT'S OUTPUT ------------------------------------------------------------

# Grasp position definition (experimentally determined position for grasping the cube)
pre_grasp_position = [-0.4, -0.314, -0.501, 0.0, 0.017, 0.8]
grasp_position = [-0.4, -0.314, -0.501, 0.0, 0.017, 0.04]

# Middle positions definition (position to lift the cube after grasping)
middle_position1 = [-0.4, 0.0, 0.0, 0.0, 0.0, 0.04]
middle_position2 = [0.4, 0.0, 0.0, 0.0, 0.0, 0.04]

# Final position definition (position to place the cube after grasping)
pre_final_position = [0.4, 0.314, -0.501, 0.0, -0.017, 0.04]
final_position = [0.4, 0.314, -0.501, 0.0, -0.017, 0.8]

# ---------------------------------------------------------------------------------------------------------------------

# STATES OF MOVEMENT:

def ToState1():     # Travel to initial position with velocity control
    current_position = EEPositionReader()
    joint_velocity = compute_velocities(current_position, initial_position)

    # Publish joint velocity to robot

def ToState2():     # Travel to grasp position with velocity control
    current_position = EEPositionReader()
    joint_velocity = compute_velocities(current_position, pre_grasp_position)

    # Publish joint velocity to robot

def ToState3():     # Grasp the cube
    current_position = EEPositionReader()
    joint_velocity = compute_velocities(current_position, grasp_position)

# Travel to middle position 2
current_position = EEPositionReader()
joint_velocity = compute_velocities(current_position, middle_position1)
current_position = EEPositionReader()
joint_velocity = compute_velocities(current_position, middle_position2)

# Travel to final position
current_position = EEPositionReader()
joint_velocity = compute_velocities(current_position, pre_final_position)
current_position = EEPositionReader()
joint_velocity = compute_velocities(current_position, final_position)





