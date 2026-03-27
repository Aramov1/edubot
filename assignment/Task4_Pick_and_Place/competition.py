import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np



class Competition(Node):
    def __init__(self, hz=10):
        super().__init__('competition')
        self.hz = 1
        self.counter = 0
        
        # --- FULL POSITIONS DICTIONARY ---
        # Format: [q1, q2, q3, q4, q5, gripper]
        self.positions = {
            "HOME": [0.0, 0.0, 0.0, -1.5, 0.0, 0.4],
            
            # Sequence 1
            "PICK_1_UP_OPEN": [-0.313, 0.501, -0.809, -1.262, -0.313, 0.4],
            "PICK_1_DOWN_OPEN": [-0.312, 0.428, -0.909, -1.09, -0.312, 0.4],
            "PICK_1_DOWN_CLOSE": [-0.312, 0.428, -0.909, -1.09, -0.312, 0.3],
            "PICK_1_UP_CLOSE": [-0.313, 0.501, -0.809, -1.262, -0.313, 0.3],
            "PLACE_1_UP_CLOSE": [-0.914, 0.428, -0.752, -1.247, -0.914, 0.3],
            "PLACE_1_DOWN_CLOSE": [-0.914, 0.357, -0.849, -1.079, -0.914, 0.3],
            "PLACE_1_DOWN_OPEN": [-0.914, 0.357, -0.849, -1.079, -0.914, 0.4],
            "PLACE_1_UP_OPEN": [-0.914, 0.428, -0.752, -1.247, -0.914, 0.4],
            
            # Sequence 2
            "PICK_2_UP_OPEN": [0.159, 0.548, -0.845, -1.273, 0.159, 0.4],
            "PICK_2_DOWN_OPEN": [0.159, 0.476, -0.949, -1.098, 0.159, 0.4],
            "PICK_2_DOWN_CLOSE": [0.159, 0.476, -0.949, -1.098, 0.159, 0.3],
            "PICK_2_UP_CLOSE": [0.159, 0.548, -0.845, -1.273, 0.159, 0.3],
            "PLACE_2_UP_CLOSE": [-0.914, 0.466, -0.632, -1.406, -0.914, 0.3],
            "PLACE_2_DOWN_CLOSE": [-0.914, 0.428, -0.752, -1.247, -0.914, 0.3],
            "PLACE_2_DOWN_OPEN": [-0.914, 0.428, -0.752, -1.247, -0.914, 0.4],
            "PLACE_2_UP_OPEN": [-0.914, 0.466, -0.632, -1.406, -0.914, 0.4],
            
            # Sequence 3 (Final Stack/Move)
            "PICK_3_UP_OPEN": [0.57, 0.343, -0.677, -1.237, 0.57, 0.4],
            "PICK_3_DOWN_OPEN": [0.57, 0.343, -0.677, -1.237, 0.57, 0.4],
            "PICK_3_DOWN_CLOSE": [0.57, 0.343, -0.677, -1.237, 0.57, 0.3],
            "PICK_3_UP_CLOSE": [0.57, 0.343, -0.677, -1.237, 0.57, 0.3],
            "PLACE_3_UP_CLOSE": [-0.914, 0.476, -0.491, -1.556, -0.914, 0.3],
            "PLACE_3_DOWN_CLOSE": [-0.914, 0.466, -0.632, -1.406, -0.914, 0.3],
            "PLACE_3_DOWN_OPEN": [-0.914, 0.466, -0.632, -1.406, -0.914, 0.4],
            "PLACE_3_UP_OPEN": [-0.914, 0.476, -0.491, -1.556, -0.914, 0.4],
            "END_HOME": [0.0, 0.0, 0.0, -1.5, 0.0, 0.4]
        }

        # --- Variables ---
        self.state_keys = list(self.positions.keys())
        self.current_state_idx = 0
        self.current_q = None
        
        # Define your specific joint names here
        self.joint_names = ['Shoulder_Rotation', 'Shoulder_Pitch', 'Elbow', 'Wrist_Pitch', 'Wrist_Roll', 'Gripper']

        # --- ROS Setup ---
        self.pub = self.create_publisher(JointTrajectory, 'joint_cmds', 10)
        self.sub = self.create_subscription(JointState, 'joint_states', self._on_state, qos_profile_sensor_data)
        self.timer = self.create_timer(1.0/self.hz, self._control_loop) 

        self.get_logger().info("PickPlaceNode initialized and waiting for state...")

    def _on_state(self, msg):
        # Create a map for quick lookup of joint positions by name
        mapping = dict(zip(msg.name, msg.position))
        self.current_q = np.array([mapping.get(name, 0.0) for name in self.joint_names])

    def _control_loop(self):
        if self.current_q is None:
            self.get_logger().warn("lost")
            return

        self.get_logger().info("next")

        # Get current target based on state machine index
        state_name = self.state_keys[self.current_state_idx]
        self.get_logger().info(f"TARGET:{state_name}")
        target_q = np.array(self.positions[state_name])
        self.get_logger().info(f"{target_q}")

        # Check Euclidean distance (error) between current and target joints
        error = np.linalg.norm(target_q - self.current_q, ord=1)        
        self.get_logger().info(f"error: {error}")
        # Transition Logic
        if error < 0.3 and self.counter == 2:  # Threshold for "Arrival"
            if self.current_state_idx < len(self.state_keys) - 1:
                self.current_state_idx += 1
                self.counter = 0
                next_state = self.state_keys[self.current_state_idx]
                self.get_logger().info(f"Reached {state_name}. Moving to {next_state}")
            else:
                self.get_logger().info("Sequence Finished!", once=True)

        self._publish_position(target_q)
        self.counter +=1
        

    def _publish_position(self, q_target):
        msg = JointTrajectory()
        #msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = q_target.tolist()
        # Set travel time to match the timer frequency
        point.time_from_start.nanosec = int((1.0/self.hz) * 1e9)
        
        msg.points = [point]
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = Competition()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

