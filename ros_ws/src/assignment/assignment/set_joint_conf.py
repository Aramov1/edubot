import numpy as np

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint # Topic to publish joint cmds

# Limited Joint Conf
JOINT_CONFIG_LIMITS = {
    'Shoulder_Rotation': (-2.0, 2.0),
    'Shoulder_Pitch':    (-np.pi/2, np.pi/2),
    'Elbow':             (-np.pi/2, np.pi/2),
    'Wrist_Pitch':       (-np.pi/2, np.pi/2),
    'Wrist_Roll':        (-np.pi, np.pi),
    'Gripper':           (0.0, np.pi)
}

class SetJointConf(Node):
    def __init__(self, verbose = False):
        super().__init__('set_joint__conf')
        self.verbose = verbose

        self.joint_names = list(JOINT_CONFIG_LIMITS.keys())
        self.target_positions = [0.0] * 6

        self.publisher = self.create_publisher(JointTrajectory, 'joint_cmds', 10)
        self.create_timer(0.1, self.publish_joint_state) # 10Hz publishing (Non-blocking)
        self.get_logger().info("Set Joint Configuration Ready.")

    def publish_joint_state(self):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = self.target_positions
        msg.points = [point]
        self.publisher.publish(msg)

    def update_target(self, new_positions):

        # Validate number of inputs
        if len(new_positions) != 6:
            print(f"Joint Configuration Refused. Expected 6 values, got {len(new_positions)}")
            return False
        
        # Validate input is within bounds
        for i, val in enumerate(new_positions):
            name = self.joint_names[i]
            low, high = JOINT_CONFIG_LIMITS[name]
            if not (low <= val <= high):
                self.get_logger().warn(f"Joint Configuration Refused. {name} ({val:.2f}) out of bounds.")
                return False
            
        self.target_positions = new_positions
        
        if self.verbose:
            print(f" Target Updated: {[round(p, 3) for p in self.target_positions]}")
        return True


def print_header():
    print(f"\n{'='*50}")
    print(f"{'BAYMAX INTERACTIVE CONTROL':^50}")
    print(f"{'='*50}")
    print(f"{'Joint Name':<20} | {'Min (rad)':>8} | {'Max (rad)':>8}")
    print(f"{'-'*20}-|-{'-'*8}-|-{'-'*8}")
    for name, (low, high) in JOINT_CONFIG_LIMITS.items():
        print(f"{name:<20} | {low:>8.2f} | {high:>8.2f}")
    print(f"{'='*50}")
    print(" Commands: [6 values (presented order)] | 'home' | 'exit'")
    print(f"{'='*50}")


def main():
    rclpy.init()
    node = SetJointConf(verbose=True)
    print_header()

    try:
        while rclpy.ok():
            # Spin once to handle the timer/publishing
            rclpy.spin_once(node, timeout_sec=0.1)
            
            # Simple terminal input
            user_input = input("\n> ").strip().lower()
            
            if user_input in ['exit', 'quit']:
                break
            elif user_input == 'home':
                node.update_target([0.0] * 6)
            elif user_input:
                try:
                    vals = [float(x) for x in user_input.split()]
                    node.update_target(vals)
                except ValueError:
                    print(" Invalid input. Enter 6 numbers separated by spaces.")
                    
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()