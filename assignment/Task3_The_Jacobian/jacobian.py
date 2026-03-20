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
    #sp.pprint(symbolic_jacobian)

    EE_pose_targets = [
        [0.2, 0.2, 0.2, 0.0, 1.57, 0.650],
        [0.2, 0.1, 0.4, 0.0, 0.0, -1.57],
        [0.0, 0.0, 0.45, 0.0, -0.785, 1.57],
        [0.0, 0.0, 0.07, 3.141, 0.0, 0.0],
        [0.0, 0.0452, 0.45, -0.785, 0.0, 3.141],
    ]

    Jacobians = []
    for target in EE_pose_targets:
        tx, ty, tz = target[0], target[1], target[2]
        pitch, roll, yaw = target[3], target[4], target[5]

        solutions = robot.inverse_kinematics(target)

        print(f"\n================================================================")
        print(f"[Test Target EE Position]    : "
              f"x={tx:.4f} m  y={ty:.4f} m  z={tz:.4f} m  "
              f"pitch={pitch:.4f} rad  roll={roll:.4f} rad  yaw={yaw:.4f} rad")

        if not solutions:
            print(f"[No IK solution found]")
            print(f"================================================================\n")
            continue

        for i, sol in enumerate(solutions, start=1):
            sol_rounded = [round(a, 4) for a in sol]
            print(f"\n[Computed Joint Angles ][{i}]: {sol_rounded}")

            # Compute and print the Jacobian for this IK solution
            jacobian_sol = robot.jacobian(*sol)
            jacobian_sol_rounded = np.round(jacobian_sol, 4)
            Jacobians.append(jacobian_sol)
            print(f"[Jacobian              ][{i}]:")
            sp.pprint(jacobian_sol_rounded)

            # Singular values act as the eigenvalue analogue for non-square Jacobians (6x5)
            singular_values = np.linalg.svd(jacobian_sol, compute_uv=False)
            singular_values_rounded = np.round(singular_values, 4)
            print(f"[Singular Values       ][{i}]: {singular_values_rounded.tolist()}")

        print(f"================================================================\n")
    return Jacobians


if __name__ == "__main__":
    main()