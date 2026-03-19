import sys
import os
_ASSIGNMENT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_PROJECT_ROOT   = os.path.dirname(_ASSIGNMENT_DIR)
sys.path.insert(0, _ASSIGNMENT_DIR)
sys.path.insert(0, os.path.join(_PROJECT_ROOT, 'ros_ws', 'src', 'python_controllers'))

import numpy as np
import sympy as sp
from robot_kinematics import RobotKinematics


def main():
    robot = RobotKinematics()

    # Print symbolic Jacobian matrix
    symbolic_jacobian = robot._symbolic_jacobian
    print("Symbolic Jacobian Matrix:")
    sp.pprint(symbolic_jacobian)
   
    #print(sp.latex(symbolic_jacobian))

    EE_pose_targets = [
        [0.2, 0.2, 0.2, 0.0, 1.57, 0.650],
        [0.2, 0.1, 0.4, 0.0, 0.0, -1.57],
        [0.0, 0.0, 0.4, 0.0, -0.785, 1.57],
        [0.0, 0.0, 0.07, 3.141, 0.0, 0.0],
        [0.0, 0.0452, 0.45, -0.785, 0.0, 3.141],
    ]

    Jacobians = []
    for target in EE_pose_targets:
        solutions = robot.inverse_kinematics(target) 
        if solutions:
            jacobian_sol = robot.jacobian(*solutions[0]) # Use first IK solution for Jacobian
            
            jacobian_sol_rounded = np.round(jacobian_sol, 4)
            Jacobians.append(jacobian_sol)
            print(f"Jacobian for target {target}: ")
            sp.pprint(jacobian_sol_rounded)
        else:
            print(f"No IK solution found for target {target}, skipping Jacobian computation.")
    return Jacobians


if __name__ == "__main__":
    main()