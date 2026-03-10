import sys
import os
_ASSIGNMENT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_PROJECT_ROOT   = os.path.dirname(_ASSIGNMENT_DIR)
sys.path.insert(0, _ASSIGNMENT_DIR)
sys.path.insert(0, os.path.join(_PROJECT_ROOT, 'ros_ws', 'src', 'python_controllers'))

import subprocess
import time
import numpy as np
import sympy as sp
import rclpy
from rclpy.node import Node
from robot_kinematics import RobotKinematics
from python_controllers.set_joint_position import JointStateSetter
from python_controllers.read_EE_position import EEPositionReader


def main():
    # Initialize the robot kinematics
    robot = RobotKinematics()
    symbolic_EE_pose = robot._symbolic_EE_pose

    # Output forward kinematics symbolic equations
    #print("Symbolic Forward Kinematics Pose Matrix:")
    #sp.pprint(symbolic_EE_pose)

    # Test points joint angles and compute the end-effector pose
    desired_test_joints = [  # Example target joint angles
        [0, 0, 0, 0, 0, 0],
    ]

    # Ensure robot_state_publisher is running; auto-launch (no GUI) if missing.
    rsp_proc = None
    try:
        result = subprocess.run(['ros2', 'node', 'list'],
                                capture_output=True, text=True, timeout=5)
        running_nodes = result.stdout
    except (subprocess.TimeoutExpired, FileNotFoundError):
        running_nodes = ''

    if 'robot_state_publisher' not in running_nodes:
        print("[INFO] robot_state_publisher not running — launching it now...")
        rsp_proc = subprocess.Popen(
            ['ros2', 'launch', 'lerobot', 'rviz.launch.py', 'use_rviz:=false'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        # Wait up to 10 s for the node to appear
        for _ in range(20):
            time.sleep(0.5)
            try:
                r = subprocess.run(['ros2', 'node', 'list'],
                                   capture_output=True, text=True, timeout=5)
                if 'robot_state_publisher' in r.stdout:
                    break
            except (subprocess.TimeoutExpired, FileNotFoundError):
                pass
        else:
            print("[ERROR] robot_state_publisher did not start in 10 s.")
            print("        Run manually: ros2 launch lerobot rviz.launch.py")
        print("[OK] robot_state_publisher is running.\n")
    else:
        print("[OK] robot_state_publisher is running.\n")

    rclpy.init()
    node = Node('fw_kinematics_checker')
    setter = JointStateSetter(node)
    reader = EEPositionReader(node)

    # Warm up: spin to receive tf_static (fixed joints publish once at startup).
    # Without this the first lookup always fails because the buffer is empty.
    print("Waiting for TF tree... ", end='', flush=True)
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.1)
    print("done.\n")

    for pose in desired_test_joints:
        arm_joints = pose[:5]
        solution = robot.forward_kinematics(*arm_joints)

        # Set joint positions in RViz via robot_state_publisher
        setter.set(pose)

        # Read EE pose from TF: [x, y, z, pitch_deg, roll_deg]
        rviz_pose = reader.read()
        if rviz_pose is None:
            print("[WARNING] TF lookup failed. Is robot_state_publisher running?")

        # Print FK result vs RViz observation
        sol = solution.flatten()   # (5,1) → (5,) to avoid numpy scalar deprecation
        fk_pitch_deg = np.rad2deg(sol[3])
        fk_roll_deg  = np.rad2deg(sol[4])

        print(f"\n================================================================")
        print(f"[Test Joint Angles]          : {pose}")
        print(f"[Computed End-Effector Pose] : "
              f"x={sol[0]:.4f}  y={sol[1]:.4f}  "
              f"z={sol[2]:.4f}  pitch={fk_pitch_deg:.2f}°  roll={fk_roll_deg:.2f}°")
        if rviz_pose is not None:
            print(f"[RVIZ End-Effector Pose]     : "
                  f"x={rviz_pose[0]:.4f}  y={rviz_pose[1]:.4f}  "
                  f"z={rviz_pose[2]:.4f}  pitch={rviz_pose[3]:.2f}°  roll={rviz_pose[4]:.2f}°")
        else:
            print(f"[RVIZ End-Effector Pose]     : [TF lookup failed]")
        print(f"================================================================\n")

    node.destroy_node()
    rclpy.shutdown()
    if rsp_proc is not None:
        rsp_proc.terminate()


if __name__ == "__main__":
    main()
