import sys
import os
_ASSIGNMENT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_PROJECT_ROOT   = os.path.dirname(_ASSIGNMENT_DIR)
sys.path.insert(0, _ASSIGNMENT_DIR)
sys.path.insert(0, os.path.join(_PROJECT_ROOT, 'ros_ws', 'src', 'python_controllers'))

import re
import numpy as np
import sympy as sp
from robot_kinematics import RobotKinematics


def compute_velocities(current_position, target):
    robot = RobotKinematics()

    # Declare singularity-related variables
    singularity_threshold=0.05
    lambda_max=0.1

    # Compute symbolic Jacobian matrix
    symbolic_jacobian = robot._symbolic_jacobian

    solution = robot.inverse_kinematics(target.tolist())
    if not solution:
        print(f"No IK solution found for target {target.tolist()}, skipping Jacobian computation.")
        return

    jacobian = robot.jacobian(*solution[0])  # Use first IK solution for Jacobian

    # Compute a constant end-effector velocity (initial position of end-effector is supposed to be 0)
    travel_time = 5.0  # seconds
    velocity = (current_position - target) / travel_time

    # Compute SVD decomposition of the Jacobian
    U, singular_values, Vt = np.linalg.svd(jacobian)

    # Singularity analysis - High condition values/low eigenvalues indicate proximity to a singularity.
    sigma_min = singular_values[-1]
    is_singular = sigma_min < singularity_threshold

    # Prepare adaptive DLS damping (Nakamura–Hanafusa 1986) ---
    if is_singular:
        # λ² scales from 0 (at threshold) to lambda_max² (at σ=0)
        lambda_sq = lambda_max**2 * (1.0 - (sigma_min / singularity_threshold)**2)
    else:
        lambda_sq = 0.0
    

    max_value = np.max(jacobian)
    if max_value > 5:
        print(f"Warning: Near singularity detected (max Jacobian value = {max_value:.4f}). "
              f"Risk of Baymaz getting stuck."
              f"Aborting movement to final position.")
    else:
        # Compute the Jacobian inverse for each target velocity
        jacobian_inv = np.linalg.pinv(jacobian)  # Use pseudo-inverse for stability

        # Compute the required joint velocities to achieve the target end-effector velocities
        joint_velocity = jacobian_inv @ velocity
        joint_velocity_rounded = np.round(joint_velocity, 4)
    
    return joint_velocity_rounded

if __name__ == "__main__":
    current_position = np.zeros(6, dtype=float)
    text = input("Enter end-effector desired position and orientation (x, y, z, rx, ry, rz): ")
    parts = [p for p in re.split(r"[\s,]+", text.strip()) if p]
    if len(parts) != 6:
        raise ValueError(f"Expected 6 values, got {len(parts)}: {parts}")
    target = np.array([float(x) for x in parts], dtype=float)

    compute_velocities(current_position, target)