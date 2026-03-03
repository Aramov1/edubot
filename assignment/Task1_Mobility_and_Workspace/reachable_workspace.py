import numpy as np
import matplotlib.pyplot as plt


def rotation_matrix(theta: float, axis: str) -> np.ndarray:
    """
    Returns the rotation matrix for a given angle and axis.

    Args:
        theta (float): The angle of rotation in radians.
        axis (str): The axis of rotation ('x', 'y', or 'z').
    """

    if axis == 'x':
        R = np.array([[1, 0, 0],
                      [0, np.cos(theta), -np.sin(theta)],
                      [0, np.sin(theta), np.cos(theta)]])

    elif axis == 'y':
        R = np.array([[np.cos(theta), 0, np.sin(theta)],
                      [0, 1, 0],
                      [-np.sin(theta), 0, np.cos(theta)]])
    else:
        R = np.array([[np.cos(theta), -np.sin(theta), 0],
                      [np.sin(theta), np.cos(theta), 0],
                      [0, 0, 1]])

    return R


def homogenous_transform(R: np.ndarray, d: np.ndarray) -> np.ndarray:
    """ 
    Returns the homogenous transformation matrix for rotation matrix and distance vector.

    Args:
        R (np.ndarray): The rotation matrix.
        d (np.ndarray): The distance vector.
    """
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = d
    T[3, :3] = [0, 0, 0]

    return T    

theta_1 = np.linspace(-2, 2, 15)
theta_2 = np.linspace(-np.pi/2, np.pi/2, 15)
theta_3 = np.linspace(-np.pi/2, np.pi/2, 15)
theta_4 = np.linspace(-np.pi/2, np.pi/2, 15)

d_world_base = np.array([0, 0, 0])
d_base_shoulder = np.array([0, -0.0452, 0.0165])
d_shoulder_upperarm = np.array([0, -0.0306, 0.1025])
d_upperarm_lowerarm = np.array([0.11257, -0.028, 0])
d_lowerarm_wrist = np.array([0.0052, -0.1349, 0])
d_wrist_gripper = np.array([-0.0601, 0, 0])
d_gripper_gripper_center = np.array([0.0, 0.0, 0.075])

x = []
y = []
z = []

R_world_base = rotation_matrix(np.pi, 'z')
T_world_base = homogenous_transform(R_world_base, d_world_base)
R_wrist_gripper = rotation_matrix(-np.pi/2, 'y')
T_wrist_gripper = homogenous_transform(R_wrist_gripper, d_wrist_gripper)
R_gripper_gripper_center = np.eye(3)
T_gripper_gripper_center = homogenous_transform(R_gripper_gripper_center, d_gripper_gripper_center)

for t1 in theta_1:
    R_base_shoulder = rotation_matrix(t1, 'z')
    T_base_shoulder = homogenous_transform(R_base_shoulder, d_base_shoulder)
    for t2 in theta_2:
        R_shoulder_upperarm = rotation_matrix(-np.pi/2+t2, 'y')
        T_shoulder_upperarm = homogenous_transform(R_shoulder_upperarm, d_shoulder_upperarm)
        for t3 in theta_3:
            R_upperarm_lowerarm = rotation_matrix(t3, 'y')
            T_upperarm_lowerarm = homogenous_transform(R_upperarm_lowerarm, d_upperarm_lowerarm)
            for t4 in theta_4:
                R_lowerarm_wrist = rotation_matrix(np.pi/2, 'z') @ rotation_matrix(t4, 'y')
                T_lowerarm_wrist = homogenous_transform(R_lowerarm_wrist, d_lowerarm_wrist)
                
                T_world_gripper_center = T_world_base @ T_base_shoulder @ T_shoulder_upperarm @ T_upperarm_lowerarm @ T_lowerarm_wrist @ T_wrist_gripper @ T_gripper_gripper_center
                x.append(T_world_gripper_center[0, 3])
                y.append(T_world_gripper_center[1, 3])
                z.append(T_world_gripper_center[2, 3])


plt.figure(figsize=(10, 8))
ax = plt.axes(projection='3d')

# 1. VISUALS: Colormap based on Z-height, smaller size (s), and transparency (alpha)
# 'plasma' or 'viridis' are great, modern color maps for scientific data
scatter = ax.scatter(x, y, z, c=z, cmap='plasma', s=2, alpha=0.15)

# Add a colorbar to the side to show what the colors mean
plt.colorbar(scatter, ax=ax, label='Height (Z-axis in meters)', shrink=0.6, pad=0.1)

# 2. CONTEXT: Mark the absolute origin (0, 0, 0) so you know where the robot's base is
ax.scatter(0, 0, 0, color='red', s=100, marker='*', label='Robot Base')

ax.set_xlabel('X-axis (m)')
ax.set_ylabel('Y-axis (m)')
ax.set_zlabel('Z-axis (m)')
ax.set_title('Reachable Workspace of the Robotic Arm')
ax.legend()

# 3. PROPORTIONS: Force equal aspect ratio so the workspace isn't stretched
max_range = np.array([max(x)-min(x), max(y)-min(y), max(z)-min(z)]).max() / 2.0
mid_x = (max(x)+min(x)) * 0.5
mid_y = (max(y)+min(y)) * 0.5
mid_z = (max(z)+min(z)) * 0.5

ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, mid_y + max_range)
ax.set_zlim(mid_z - max_range, mid_z + max_range)

plt.tight_layout()
plt.show()



