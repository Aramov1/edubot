import sys
import os
_ASSIGNMENT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_PROJECT_ROOT   = os.path.dirname(_ASSIGNMENT_DIR)
sys.path.insert(0, _ASSIGNMENT_DIR)
sys.path.insert(0, os.path.join(_PROJECT_ROOT, 'ros_ws', 'src', 'assignment'))

import subprocess
import time
import numpy as np
import rclpy
from rclpy.node import Node
from robot_kinematics import RobotKinematics
from assignment.joint_state_setter import JointStateSetter
from assignment.read_ee_position import EEPositionReader


def main():
    # Create instance of RobotKinematics to access FK and IK methods
    robot = RobotKinematics()

    # Define EE pose targets: [x, y, z, pitch, roll, yaw]
    EE_pose_targets = [
        [0.2, 0.2, 0.2, 0.0, 1.57, 0.650],
        [0.2, 0.1, 0.4, 0.0, 0.0, -1.57],
        [0.0, 0.0, 0.45, 0.0, -0.785, 1.57],
        [0.0, 0.0, 0.07, 3.141, 0.0, 0.0],
        [0.0, 0.0452, 0.45, -0.785, 0.0, 3.141],
    ]

    # Ensure robot_state_publisher is running; auto-launch (no GUI) if missing.
    # robot_state_publisher should be running in order to be able to read EE pose from RVIZ
    rsp_proc = None
    try:
        result = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True, timeout=5)
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


    # Initiate setter and reader ros2 nodes
    rclpy.init()
    node = Node('inv_kinematics_checker')
    setter = JointStateSetter(node)
    reader = EEPositionReader(node)

    # Warm up: spin to receive tf_static (fixed joints publish once at startup).
    print("Waiting for TF tree... ", end='', flush=True)
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.1)
    print("done.\n")

    # Evaluate IK for a set of target EE poses
    print("--- Inverse Kinematics ---")
    for target in EE_pose_targets:
        solutions = robot.inverse_kinematics(target)

        tx, ty, tz = target[0], target[1], target[2]
        pitch, roll, yaw = target[3], target[4], target[5]

        print(f"\n================================================================")
        print(f"[Test Target EE Position]    : "
              f"x={tx:.4f} m  y={ty:.4f} m z={tz:.4f}  m "
              f"pitch={pitch:.4f} rad  roll={roll:.4f} rad  yaw={yaw:.4f} rad")

        if not solutions:
            print(f"[No IK solution found]")
            print(f"================================================================\n")
            continue

        for i, sol in enumerate(solutions, start=1):
            # Set joint angles in RViz (append gripper=0)
            setter.set([*sol, 0.0])

            # Read EE pose from TF: [x, y, z, pitch, roll, yaw] — all angles in radians
            rviz_pose = reader.read()

            # Print numerical inv kinematics solution
            sol_rounded = [round(a, 4) for a in sol]
            print(f"\n[Computed Joint Angles ][{i}]: {sol_rounded}")

            # Print RVIZ EE pose for the joint configuration computed and compute errors
            if rviz_pose is not None:
                rx, ry, rz = rviz_pose[0], rviz_pose[1], rviz_pose[2]
                dist = np.sqrt((tx - rx)**2 + (ty - ry)**2 + (tz - rz)**2)
                
                pitch_err = abs(pitch - rviz_pose[3])
                roll_err  = abs(roll  - rviz_pose[4])
                yaw_err   = abs(yaw   - rviz_pose[5])

                print(f"[RViz End-Effector Pose][{i}]: "
                      f"x={rx:.4f} m  y={ry:.4f} m  z={rz:.4f} m "
                      f"pitch={rviz_pose[3]:.4f} rad  roll={rviz_pose[4]:.4f} rad  yaw={rviz_pose[5]:.4f} rad")
                print(f"[Error                 ][{i}]: "
                      f"Dist={dist*100:.4f} cm  pitch_err={pitch_err:.4f} rad  "
                      f"roll_err={roll_err:.4f} rad  yaw_err={yaw_err:.4f} rad")
            else:
                print(f"[RViz End-Effector Pose][{i}]: [TF lookup failed]")

        print(f"================================================================\n")

    node.destroy_node()
    rclpy.shutdown()
    if rsp_proc is not None:
        rsp_proc.terminate()


if __name__ == "__main__":
    main()
