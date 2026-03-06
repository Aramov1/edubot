import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import sympy as sp
import matplotlib.pyplot as plt
from  robot_kinematics import RobotKinematics


def main():
    # Create instance of RobotKinematics to access FK and IK methods
    robot = RobotKinematics()

    # Evaluate IK for a set of target EE poses (x, y, z, pitch, roll)
    print("--- Inverse Kinematics ---")

    # Define EE pose targets: [x, y, z, pitch, roll]
    EE_pose_targets = [
        [0.2, 0.2, 0.2, 1.57, 0.0],
        [0.2, 0.1, 0.4, 0.0, 1.57],
        [0.0, 0.0, 0.45, 0.785, 0.785],
        [0.0, 0.0, 0.07, 3.141, 0.0],
        [0.0, 0.0452, 0.45, 0.785, 3.141],
    ]

    for target in EE_pose_targets:
        solutions = robot.inverse_kinematics(target)

        if not solutions:
            res_str = "No solution found"
        else:
            res_str = [[round(a, 4) for a in sol] for sol in solutions]

        print(f"{str(np.round(target, 3)):<45} | {res_str}")
    

if __name__ == "__main__":
    main()


