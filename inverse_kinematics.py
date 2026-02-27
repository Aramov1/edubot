import matplotlib.pyplot as plt
import sympy as np

# 1. Declaration of the variables
t1 = np.Symbol('t1')
t2 = np.Symbol('t2')
t3 = np.Symbol('t3')
t4 = np.Symbol('t4')
t5 = np.Symbol('t5') # This angle will be the one of the jaw, which is the same as the roll of the end-effector

# 2. Computation of the homogenous transform
def rotation_matrix(theta, axis: str):
    """
    Returns the rotation matrix for a given angle and axis.

    Args:
        theta (float): The angle of rotation in radians.
        axis (str): The axis of rotation ('x', 'y', or 'z').
    """
    if axis == 'x':
        R = np.Matrix([[1, 0, 0],
                      [0, np.cos(theta), -np.sin(theta)],
                      [0, np.sin(theta), np.cos(theta)]])

    elif axis == 'y':
        R = np.Matrix([[np.cos(theta), 0, np.sin(theta)],
                      [0, 1, 0],
                      [-np.sin(theta), 0, np.cos(theta)]])
    else:
        R = np.Matrix([[np.cos(theta), -np.sin(theta), 0],
                      [np.sin(theta), np.cos(theta), 0],
                      [0, 0, 1]])

    return R

def homogenous_transform(R, d):
    """ 
    Returns the homogenous transformation matrix for rotation matrix and distance vector.

    Args:
        R (np.ndarray): The rotation matrix.
        d (np.ndarray): The distance vector.
    """
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = d

def T_computation(theta_1, theta_2, theta_3, theta_4):
    d_world_base = np.Matrix([0, 0, 0])
    d_base_shoulder = np.Matrix([0, -0.0452, 0.0165])
    d_shoulder_upperarm = np.Matrix([0, -0.0306, 0.1025])
    d_upperarm_lowerarm = np.Matrix([0.11257, -0.028, 0])
    d_lowerarm_wrist = np.Matrix([0.0052, -0.1349, 0])
    d_wrist_gripper = np.Matrix([-0.0601, 0, 0])
    d_gripper_gripper_center = np.Matrix([0.0, 0.0, 0.075])

    R_world_base = rotation_matrix(np.pi, 'z')
    T_world_base = homogenous_transform(R_world_base, d_world_base)
    R_wrist_gripper = rotation_matrix(-np.pi/2, 'y')
    T_wrist_gripper = homogenous_transform(R_wrist_gripper, d_wrist_gripper)
    R_gripper_gripper_center = np.eye(3)
    T_gripper_gripper_center = homogenous_transform(R_gripper_gripper_center, d_gripper_gripper_center)
    
    R_base_shoulder = rotation_matrix(theta_1, 'z')
    T_base_shoulder = homogenous_transform(R_base_shoulder, d_base_shoulder)
    R_shoulder_upperarm = rotation_matrix(-np.pi/2+theta_2, 'y')
    T_shoulder_upperarm = homogenous_transform(R_shoulder_upperarm, d_shoulder_upperarm)
    R_upperarm_lowerarm = rotation_matrix(theta_3, 'y')
    T_upperarm_lowerarm = homogenous_transform(R_upperarm_lowerarm, d_upperarm_lowerarm)
    R_lowerarm_wrist = rotation_matrix(np.pi/2, 'z') @ rotation_matrix(theta_4, 'y')
    T_lowerarm_wrist = homogenous_transform(R_lowerarm_wrist, d_lowerarm_wrist)
    T_world_gripper_center = T_world_base * T_base_shoulder * T_shoulder_upperarm * T_upperarm_lowerarm * T_lowerarm_wrist * T_wrist_gripper * T_gripper_gripper_center

    return T_world_gripper_center

T = T_computation(t1, t2, t3, t4)

np.pprint(T)

