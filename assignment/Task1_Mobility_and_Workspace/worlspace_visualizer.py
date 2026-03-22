import os
import sys
_ASSIGNMENT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  
_PROJECT_ROOT   = os.path.dirname(_ASSIGNMENT_DIR)                             
sys.path.insert(0, _ASSIGNMENT_DIR)                                            
sys.path.insert(0, os.path.join(_PROJECT_ROOT, 'ros_ws', 'src', 'assignment', 'assignment'))

# Updated Dependencies
import rclpy
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from rviz_draw_marker import RvizDrawMarker
from robot_kinematics import RobotKinematics

def main():

    robot = RobotKinematics()

    # Initialize Nodes
    rclpy.init()
    marker = RvizDrawMarker(verbose=True)

    while rclpy.ok() and not marker.tf_found:
        rclpy.spin_once(marker, timeout_sec=0.5)

    print(f"\n{'='*60}\n WORKSPACE VISUALIZER START\n{'='*60}")
    
    try:
        # Number of joint sampling resolution
        res = 15

        # Extract joint bounds
        spaces = [np.linspace(b[0], b[1], res) for b in robot.joint_bounds]
        
        # Create grid for all joint combinations
        grids = np.meshgrid(*spaces, indexing='ij')

        # Flatten the grid
        q_flattened = [g.flatten() for g in grids]

        # Compute Forward Kinematics
        pose = robot.forward_kinematics(q_flattened)
        
        # Extract X, Y, Z and Publish
        x_pts, y_pts, z_pts = pose[0], pose[1], pose[2]
        marker.publish_workspace(x_pts, y_pts, z_pts)
        
    except KeyboardInterrupt:
        pass
    finally:
        marker.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()


