import os
import sys
_ASSIGNMENT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  
_PROJECT_ROOT   = os.path.dirname(_ASSIGNMENT_DIR)                             
sys.path.insert(0, _ASSIGNMENT_DIR)                                            
sys.path.insert(0, os.path.join(_PROJECT_ROOT, 'ros_ws', 'src', 'assignment', 'assignment'))

import rclpy
import time
import numpy as np

# Updated Dependencies
from set_joint_conf import SetJointConf
from read_ee_cart import ReadEECart
from robot_kinematics import RobotKinematics

def main():

    robot = RobotKinematics()
    
    # Initialize Nodes
    rclpy.init()
    setter = SetJointConf()
    reader = ReadEECart()

    while rclpy.ok() and not reader.tf_found:
        rclpy.spin_once(reader, timeout_sec=0.5)
        rclpy.spin_once(setter, timeout_sec=0.0)

    # Test points: [x, y, z, rot_X, rot_y, rot_z]
    ee_pose_to_test = [
        [0.2, 0.2, 0.2, 0.0, 1.57, 0.650],
        [0.2, 0.1, 0.4, 0.0, 0.0, -1.57],
        [0.0, 0.0, 0.45, 0.0, -0.785, 1.57],
        [0.0, 0.0, 0.07, 3.141, 0.0, 0.0],
        [0.0, 0.0452, 0.45, -0.785, 0.0, 3.141],
    ]

    print(f"\n{'='*65}\n INVERSE KINEMATICS VALIDATION\n{'='*65}")

    try:
        for ee_pose in ee_pose_to_test:
            print(f"\nTesting EE POSE: POS (m) x={ee_pose[0]:.3f} y={ee_pose[1]:.3f} z={ee_pose[2]:.3f} "
                  f"ROT (rad): rot_x={ee_pose[3]:.2f} rot_y={ee_pose[4]:.2f} rot_z={ee_pose[5]:.2f}")
            
            # Find all IK solutions
            solutions = robot.inverse_kinematics(ee_pose)

            if not solutions:
                print("  [!!] No IK solutions found for this target.")
                continue


            for i, sol in enumerate(solutions, start=1):
                # Publish joint command in ROS/RViz 
                setter.update_target([*sol, 0.0])
                
                # # Fire the setter's 10 Hz timer so the command is published
                rclpy.spin_once(setter, timeout_sec=0.15)

                # Wait foe tf pipeline to complete
                time.sleep(0.1)

                # # Drain ALL pending /tf callbacks with non-blocking calls.
                for _ in range(500):
                    rclpy.spin_once(reader, timeout_sec=0.0)    

                # Read EE pose using from TF Tree
                rviz_pose = reader.get_ee_pose()

                if rviz_pose:
                    # Calculate errors
                    error_pos = np.linalg.norm(np.array(ee_pose[:3]) - np.array(rviz_pose[:3]))
                    error_rot = np.minimum(abs(np.array(ee_pose[3:]) - np.array(rviz_pose[3:])), abs(np.array(rviz_pose[3:]) - np.array(ee_pose[3:])))

                    print(f"  Sol:     {i}: {np.round(sol, 3)}")
                    print(f"  RVIZ:    POS (m): x={rviz_pose[0]:.3f}, y={rviz_pose[1]:.3f}, z={rviz_pose[2]:.3f}  ROT (rad): rot_x={rviz_pose[3]:.3f}, rot_y={rviz_pose[4]:.3f}, rot_z={rviz_pose[5]:.3f}")
                    print(f"  ERROR:   POS (cm): {error_pos*100:.5f}  ROT (rad): rot_x={error_rot[0]:.3f}, rot_y={error_rot[1]:.3f}, rot_z={error_rot[2]:.3f}\n")
                else:
                    print(f"  Sol {i}: TF lookup failed during test.\n ")
            
            print("-" * 65)

    except KeyboardInterrupt:
        pass
    finally:
        setter.destroy_node()
        reader.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()