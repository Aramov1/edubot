import sys
import os
_ASSIGNMENT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_PROJECT_ROOT   = os.path.dirname(_ASSIGNMENT_DIR)
sys.path.insert(0, _ASSIGNMENT_DIR)
sys.path.insert(0, os.path.join(_PROJECT_ROOT, 'ros_ws', 'src', 'python_controllers'))

import subprocess
import time
import numpy as np
import rclpy
from rclpy.node import Node
from robot_kinematics import RobotKinematics
from python_controllers.set_joint_position import JointStateSetter
from python_controllers.read_EE_position import EEPositionReader


def main():
    # Create instance of RobotKinematics to access FK and IK methods
    robot = RobotKinematics()

    # Evaluate IK for a set of target EE poses (x, y, z, pitch, roll)
    print("--- Inverse Kinematics ---")

    # Define EE pose targets: [x, y, z, pitch, roll]
    EE_pose_targets = [
        [0.2, 0.2, 0.2, 0.0, 1.57, 0.650],
        [0.2, 0.1, 0.4, 0.0, 0.0, -1.57],
        [0.0, 0.0, 0.4, 0.0, -0.785, 1.57],
        [0.0, 0.0, 0.07, 3.141, 0.0, 0.0],
        [0.0, 0.0452, 0.45, -0.785, 0.0, 3.141],
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
    node = Node('inv_kinematics_checker')
    setter = JointStateSetter(node)
    reader = EEPositionReader(node)

    # Warm up: spin to receive tf_static (fixed joints publish once at startup).
    print("Waiting for TF tree... ", end='', flush=True)
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.1)
    print("done.\n")

    for target in EE_pose_targets:
        solutions = robot.inverse_kinematics(target)

        tx, ty, tz = target[0], target[1], target[2]
        pitch_deg = np.rad2deg(target[3])
        roll_deg  = np.rad2deg(target[4])
        yaw_deg   = np.rad2deg(target[5])

        print(f"\n================================================================")
        print(f"[Test Target EE Position]    : "
              f"x={tx:.4f}  y={ty:.4f}  z={tz:.4f}  "
              f"pitch={pitch_deg:.2f}°  roll={roll_deg:.2f}°  yaw={yaw_deg:.2f}°")

        if not solutions:
            print(f"[No IK solution found]")
            print(f"================================================================\n")
            continue

        for i, sol in enumerate(solutions, start=1):
            # Set joint angles in RViz (append gripper=0)
            setter.set([*sol, 0.0])

            # Read EE pose from TF: [x, y, z, pitch_deg, roll_deg]
            rviz_pose = reader.read()

            sol_rounded = [round(a, 4) for a in sol]

            print(f"\n[Computed Joint Angles ][{i}]: {sol_rounded}")

            if rviz_pose is not None:
                rx, ry, rz = rviz_pose[0], rviz_pose[1], rviz_pose[2]
                dist = np.sqrt((tx - rx)**2 + (ty - ry)**2 + (tz - rz)**2)
                pitch_err = abs(pitch_deg - rviz_pose[3])
                roll_err  = abs(roll_deg  - rviz_pose[4])
                yaw_err   = abs(yaw_deg   - rviz_pose[5])
                print(f"[RViz End-Effector Pose][{i}]: "
                      f"x={rviz_pose[0]:.4f}  y={rviz_pose[1]:.4f}  "
                      f"z={rviz_pose[2]:.4f}  pitch={rviz_pose[3]:.2f}°  roll={rviz_pose[4]:.2f}°   yaw={rviz_pose[5]:.2f}°")
                print(f"[Error                 ][{i}]: "
                      f"Dist={dist:.4f} m  pitch_err={pitch_err:.2f}°  roll_err={roll_err:.2f}°  yaw_err={yaw_err:.2f}°")
            else:
                print(f"[RViz End-Effector Pose][{i}]: [TF lookup failed]")

        print(f"================================================================\n")

    node.destroy_node()
    rclpy.shutdown()
    if rsp_proc is not None:
        rsp_proc.terminate()


if __name__ == "__main__":
    main()
