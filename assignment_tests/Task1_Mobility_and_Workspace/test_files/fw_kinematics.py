import os
import sys
_ASSIGNMENT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_PROJECT_ROOT   = os.path.dirname(_ASSIGNMENT_DIR)                             
sys.path.insert(0, _ASSIGNMENT_DIR)                                            
sys.path.insert(0, os.path.join(_PROJECT_ROOT, 'ros_ws', 'src', 'assignment', 'assignment'))

# Updated Dependencies
import subprocess
import rclpy
import time
import numpy as np

from set_joint_conf import SetJointConf
from read_ee_cart import ReadEECart
from robot_kinematics import RobotKinematics

def main():
    
    robot = RobotKinematics()

    # Kill any existing ROS2 processes and relaunch the simulation
    subprocess.run(["pkill", "-9", "-f", "ros2"], check=False)
    time.sleep(1)
  
    # Initialize Nodes
    rclpy.init()
    setter = SetJointConf()
    reader = ReadEECart()

    while rclpy.ok() and not reader.tf_found:
        rclpy.spin_once(reader, timeout_sec=0.5)
        rclpy.spin_once(setter, timeout_sec=0.0)
    
    # Test points: [q1, q2, q3, q4, q5, gripper]
    joints_to_test = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 1.57, 0.0],
        [1.57, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.5, 0.5, 0.0, 0.0, 0.0]
    ]

    print(f"\n{'='*60}\n FORWARD KINEMATICS VALIDATION START\n{'='*60}")

    try:
        for joints in joints_to_test:
            # Compute FK (Using only first 5 joints)
            computed_pose = robot.forward_kinematics(joints[:5])

            # Publish joint command in ROS/RViz
            setter.update_target(joints)

            # Fire the setter's 10 Hz timer so the command is published
            rclpy.spin_once(setter, timeout_sec=0.15)

            # Wait for tf pipeline to complete
            time.sleep(0.1)

            # # Drain ALL pending /tf callbacks with non-blocking calls.
            for _ in range(500):
                rclpy.spin_once(reader, timeout_sec=0.0)

            # Read EE pose from the now-fresh TF buffer
            measured_pose = reader.get_ee_pose()

            # Display Results
            print(f"\nTesting Joints Configuration: {joints}")
            print(f"  Computed: POS (m): x={computed_pose[0]:.3f}, y={computed_pose[1]:.3f}, z={computed_pose[2]:.3f}  ROT (rad): rot_x={computed_pose[3]:.3f}, rot_y={computed_pose[4]:.3f}, rot_z={computed_pose[5]:.3f}")
            
            if measured_pose:
                print(f"  RVIZ:     POS (m): x={measured_pose[0]:.3f}, y={measured_pose[1]:.3f}, z={measured_pose[2]:.3f}  ROT (rad): rot_x={measured_pose[3]:.3f}, rot_y={measured_pose[4]:.3f}, rot_z={measured_pose[5]:.3f}")
                # Compute Error
                error_pos = np.linalg.norm(computed_pose[:3] - np.array(measured_pose[:3]))
                error_rot = np.minimum(abs(computed_pose[3:] - measured_pose[3:]), abs(measured_pose[3:] - computed_pose[3:]))
                print(f"  ERROR:    POS (cm): {error_pos*100:.5f}  ROT (rad): rot_x={error_rot[0]:.3f}, rot_y={error_rot[1]:.3f}, rot_z={error_rot[2]:.3f}")
            else:
                print("  RVIZ:    [TF Lookup Failed! Check if robot_state_publisher is running]")
            
            print("-" * 60)

    except KeyboardInterrupt:
        pass
    finally:
        setter.destroy_node()
        reader.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()