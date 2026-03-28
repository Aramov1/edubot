import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JointCommandExecutor(Node):
    def __init__(self):
        super().__init__('joint_command_executor')
        
        self._joint_cmd_pub = self.create_publisher(JointTrajectory, 'joint_cmds', 10)
        
        # 🔧 Define your robot joint names here
        self.joint_names = [
            "base", "shoulder", "upper_arm",
            "lower_arm", "wrist", "gripper"
        ]
        
        self.trajectory = []
        self.current_idx = 0
        self.timer = None

    # ================================
    # Execute joint trajectory
    # ================================
    def execute(self, joint_waypoints):
        """
        joint_waypoints: list of dicts
        {
            "joints": [q1, q2, ..., q6],
            "time": seconds
        }
        """
        if not joint_waypoints:
            print("Empty trajectory!")
            return
        
        self.trajectory = joint_waypoints
        self.current_idx = 0
        
        print(f"Executing {len(self.trajectory)} joint waypoints...")
        
        # Start immediately
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        if self.current_idx >= len(self.trajectory):
            print("Execution complete!")
            self.timer.cancel()
            return
        
        point_data = self.trajectory[self.current_idx]
        
        joint_angles = point_data["joints"]
        duration = point_data["time"]
        
        # Add gripper value if needed
        joint_angles_with_gripper = joint_angles
        
        # ================================
        # ROS MESSAGE (Robot Communication)
        # ================================
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = list(joint_angles_with_gripper)
        point.time_from_start = rclpy.time.Duration(seconds=duration).to_msg()
        
        traj_msg.points.append(point)
        
        # SEND TO ROBOT
        self._joint_cmd_pub.publish(traj_msg)
        
        print(f"Moving to waypoint {self.current_idx} in {duration}s")
        
        self.current_idx += 1
        
        # Wait for this motion to finish before sending next
        self.timer.cancel()
        self.timer = self.create_timer(duration, self.timer_callback)


# ================================
# MAIN: Define joint deflections
# ================================
def main():
    rclpy.init()
    
    executor = JointCommandExecutor()
    
    # # USER INPUT: joint angles (radians) + time
    # joint_waypoints = [
    #     #{"joints": [0.0, 0.0, 0.0, -1.5, 0.0, 0.4], "time": 3.0},                       # HOME
    #     {"joints": [-0.31, 0.539, -0.7, -1.422, -0.31, 0.4], "time": 3.0},            # PICK 1 + OFFSET (open)
    #     {"joints": [-0.312, 0.428, -0.95, -1.09, -0.312, 0.4], "time": 2.0},           # PICK 1
    #     {"joints": [-0.312, 0.428, -0.95, -1.09, -0.312, 0.2], "time": 1.0},           # CLOSE GRIPPER
    #     {"joints": [-0.31, 0.539, -0.7, -1.422, -0.31, 0.2], "time": 2.0},            # PICK 1 + OFFSET (closed)
    #     {"joints": [-1.014, 0.454, -0.5, -1.552, -1.014, 0.2], "time": 2.0},          # PLACE + OFFSET (closed)
    #     {"joints": [-0.94, 0.357, -0.9, -1.079, -0.914, 0.2], "time": 2.0},           # PLACE 1
    #     {"joints": [-0.94, 0.357, -0.9, -1.079, -0.914, 0.4], "time": 1.0},           # OPEN GRIPPER
    #     {"joints": [-1.014, 0.454, -0.5, -1.552, -1.014, 0.4], "time": 2.0},          # PLACE + OFFSET (open)
        
    #     {"joints": [0.0, 0.0, 0.0, -1.55, 0.0, 0.4], "time": 3.0},                       # HOME
    #     {"joints": [0.159, 0.582, -0.75, -1.434, 0.159, 0.4], "time": 3.0},            # PICK 2 + OFFSET (open)
    #     {"joints": [0.159, 0.476, -2.0, -1.098, 0.159, 0.4], "time": 2.0},            # PICK 2
    #     {"joints": [0.159, 0.476, -1, -1.098, 0.159, 0.2], "time": 1.0},            # CLOSE GRIPPER
    #     {"joints": [0.159, 0.582, -0.77, -1.434, 0.159, 0.2], "time": 2.0},            # PICK 2 + OFFSET (closed)
    #     {"joints": [-1.014, 0.454, -0.523, -1.552, -1.014, 0.2], "time": 3.0},          # PLACE + OFFSET (closed)
    #     {"joints": [-0.94, 0.428, -0.8, -1.247, -0.94, 0.2], "time": 2.0},            # PLACE 2  
    #     {"joints": [-0.94, 0.428, -0.8, -1.247, -0.94, 0.4], "time": 1.0},            # OPEN GRIPPER
    #     {"joints": [-1.014, 0.454, -0.52, -1.552, -1.014, 0.4], "time": 2.0},          # PLACE + OFFSET (open)

    #     {"joints": [0.0, 0.0, 0.0, -1.55, 0.0, 0.4], "time": 3.0},                       # HOME
    #     {"joints": [0.57, 0.382, -0.6, -1.393, 0.57, 0.4], "time": 3.0},              # PICK 3 + OFFSET (open)  
    #     {"joints": [0.57, 0.273, -0.821, -1.073, 0.57, 0.4], "time": 2.0},              # PICK 3
    #     {"joints": [0.57, 0.273, -0.821, -1.073, 0.57, 0.25], "time": 1.0},              # CLOSE GRIPPER
    #     {"joints": [0.57, 0.382, -0.61, -1.393, 0.57, 0.2], "time": 2.0},              # PICK 3 + OFFSET (closed)
    #     {"joints": [-1.014, 0.454, -0.523, -1.552, -1.014, 0.2], "time": 5.0},          # PLACE + OFFSET (closed)
    #     {"joints": [-0.94, 0.466, -0.622, -1.406, -0.94, 0.2], "time": 3.0},            # PLACE 3  
    #     {"joints": [-0.94, 0.466, -0.622, -1.406, -0.94, 0.4], "time": 1.0},            # OPEN GRIPPER

    # ]

     # USER INPUT: joint angles (radians) + time
    joint_waypoints = [
        #{"joints": [0.0, 0.0, 0.0, -1.5, 0.0, 0.4], "time": 3.0},                       # HOME
        {"joints": [-0.31, 0.539, -0.7, -1.422, -0.31, 0.4], "time": 3.0},            # PICK 1 + OFFSET (open)
        {"joints": [-0.312, 0.428, -0.95, -1.09, -0.312, 0.4], "time": 2.0},           # PICK 1
        {"joints": [-0.312, 0.428, -0.95, -1.09, -0.312, 0.2], "time": 1.0},           # CLOSE GRIPPER
        {"joints": [-0.31, 0.539, -0.7, -1.422, -0.31, 0.2], "time": 2.0},            # PICK 1 + OFFSET (closed)
        {"joints": [-1.014, 0.454, -0.5, -1.552, -1.014, 0.2], "time": 2.0},          # PLACE + OFFSET (closed)
        {"joints": [-0.94, 0.357, -0.9, -1.079, -0.914, 0.2], "time": 2.0},           # PLACE 1
        {"joints": [-0.94, 0.357, -0.9, -1.079, -0.914, 0.4], "time": 1.0},           # OPEN GRIPPER
        {"joints": [-1.014, 0.454, -0.5, -1.552, -1.014, 0.4], "time": 2.0},          # PLACE + OFFSET (open)
        
            #{"joints": [0.0, 0.0, 0.0, -1.5, 0.0, 0.4], "time": 3.0},                       # HOME
        {"joints": [-0.31, 0.539, -0.7, -1.422, -0.31, 0.4], "time": 3.0},            # PICK 1 + OFFSET (open)
        {"joints": [-0.312, 0.428, -0.95, -1.09, -0.312, 0.4], "time": 2.0},           # PICK 1
        {"joints": [-0.312, 0.428, -0.95, -1.09, -0.312, 0.2], "time": 1.0},           # CLOSE GRIPPER
        {"joints": [-0.31, 0.539, -0.7, -1.422, -0.31, 0.2], "time": 2.0},            # PICK 1 + OFFSET (closed)
        {"joints": [-1.014, 0.454, -0.523, -1.552, -1.014, 0.2], "time": 3.0},          # PLACE + OFFSET (closed)
        {"joints": [-0.94, 0.428, -0.8, -1.247, -0.94, 0.2], "time": 2.0},            # PLACE 2  
        {"joints": [-0.94, 0.428, -0.8, -1.247, -0.94, 0.4], "time": 1.0},            # OPEN GRIPPER
        {"joints": [-1.014, 0.454, -0.52, -1.552, -1.014, 0.4], "time": 2.0},          # PLACE + OFFSET (open)

            #{"joints": [0.0, 0.0, 0.0, -1.5, 0.0, 0.4], "time": 3.0},                       # HOME
        {"joints": [-0.31, 0.539, -0.7, -1.422, -0.31, 0.4], "time": 3.0},            # PICK 1 + OFFSET (open)
        {"joints": [-0.312, 0.428, -0.95, -1.09, -0.312, 0.4], "time": 2.0},           # PICK 1
        {"joints": [-0.312, 0.428, -0.95, -1.09, -0.312, 0.2], "time": 1.0},           # CLOSE GRIPPER
        {"joints": [-0.31, 0.539, -0.7, -1.422, -0.31, 0.2], "time": 2.0},            # PICK 1 + OFFSET (closed)
        {"joints": [-1.014, 0.454, -0.523, -1.552, -1.014, 0.2], "time": 5.0},          # PLACE + OFFSET (closed)
        {"joints": [-0.94, 0.466, -0.622, -1.406, -0.94, 0.2], "time": 3.0},            # PLACE 3  
        {"joints": [-0.94, 0.466, -0.622, -1.406, -0.94, 0.4], "time": 1.0},            # OPEN GRIPPER

    ]
    
    executor.execute(joint_waypoints)
    
    try:
        rclpy.spin(executor)
    except KeyboardInterrupt:
        pass
    finally:
        executor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()