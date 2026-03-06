import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import sympy as sp
import matplotlib.pyplot as plt
from  robot_kinematics import RobotKinematics

def plot_workspace(x, y, z):
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    scatter = ax.scatter(x, y, z, c=z, cmap='magma', s=1, alpha=0.1)
    ax.scatter(0, 0, 0, color='cyan', s=50, marker='D', label='Origin')
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    plt.colorbar(scatter, label='Z (m)')
    ax.set_title("Reachable Workspace of the Robotic Arm")
    
    # Scale axes equally
    limit = np.max([np.abs(x).max(), np.abs(y).max(), np.abs(z).max()])
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_zlim(-limit, limit)
    plt.show()


def main():

    robot = RobotKinematics()
    symbolic_EE_pose = robot._symbolic_EE_pose

    # Output the symbolic matrix to see the math
    print("Symbolic Forward Kinematics Pose Matrix:")
    sp.pprint(symbolic_EE_pose)



    
    # Savve equations to a file. Uncomment if needed
    with open("robot_fw_kinematics.txt", "w", encoding="utf-8") as f:
        fw_kinematics_equation = sp.pretty(symbolic_EE_pose, use_unicode=False, wrap_line=False)

        f.write("FORWARD KINEMATICS SYMBOLIC EQUATIONS (X, Y, Z, Pitch, Roll)\n\n")
        f.write(fw_kinematics_equation)

    print("\n[System] Symbolic equations have been saved to 'robot_fw_kinematics.txt'")

    with open("jacobian_fw_kinematics.txt", "w", encoding="utf-8") as f:
        fw_kinematics_equation = sp.pretty(robot._symbolic_jacobian, use_unicode=False, wrap_line=False)

        f.write("FORWARD KINEMATICS JACOBIAN SYMBOLIC EQUATIONS\n\n")
        f.write(fw_kinematics_equation)

    print("\n[System] Symbolic equations have been saved to 'jacobian_fw_kinematics.txt'")
    
"""
    # Calculate and plot
    x_pts, y_pts, z_pts = robot.compute_workspace()
    plot_workspace(x_pts, y_pts, z_pts)
"""

if __name__ == "__main__":
    main()



