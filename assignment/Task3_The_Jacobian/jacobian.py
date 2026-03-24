import sys
import os
_ASSIGNMENT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ASSIGNMENT_DIR)

import numpy as np
from robot_kinematics import RobotKinematics


def main():
    robot = RobotKinematics()

    # Test points: [x, y, z, rot_X, rot_y, rot_z]
    ee_pose_to_test = [
        [0.2, 0.2, 0.2, 0.0, 1.57, 0.650],
        [0.2, 0.1, 0.4, 0.0, 0.0, -1.57],
        [0.0, 0.0, 0.45, 0.0, -0.785, 1.57],
        [0.0, 0.0, 0.07, 3.141, 0.0, 0.0],
        [0.0, 0.0452, 0.45, -0.785, 0.0, 3.141],
    ]

    print(f"\n{'='*60}\n JACOBIAN COMPUTATION VALIDATION START\n{'='*60}")


    for ee_pose in ee_pose_to_test:

        print("-" * 65)

        print(f"\nTesting EE POSE: POS (m) x={ee_pose[0]:.3f} y={ee_pose[1]:.3f} z={ee_pose[2]:.3f} "
                f"ROT (rad): rot_x={ee_pose[3]:.2f} rot_y={ee_pose[4]:.2f} rot_z={ee_pose[5]:.2f}")
        
        # Find all IK solutions
        solutions = robot.inverse_kinematics(ee_pose)

        if not solutions:
            print("  [!!] No IK solutions found for this target.\n")
            continue

        for i, sol in enumerate(solutions, start=1):
                
            jacobian = robot.jacobian(sol)
            singular_values = np.linalg.svd(jacobian, compute_uv=False)

            print(f"  Sol {i}:  {np.round(sol, 3)}")
            print(f"  Jacobian:")
            print(np.array2string(jacobian, precision=4, suppress_small=True, separator=', '))
            print(f"  Singular Values: {np.round(singular_values, 3)}")

    print("-" * 65)


if __name__ == "__main__":
    main()

