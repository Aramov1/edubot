import sys
import os
_ASSIGNMENT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_PROJECT_ROOT   = os.path.dirname(_ASSIGNMENT_DIR)
sys.path.insert(0, _ASSIGNMENT_DIR)
sys.path.insert(0, os.path.join(_PROJECT_ROOT, 'ros_ws', 'src', 'assignment'))

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from robot_kinematics import RobotKinematics


class VelTrajectoryController(Node):
    def __init__(self, start_pos, offset=0.2, gain_pos=0.6, gain_rot = 0.3, max_joint_vel=0.5, max_cart_vel = 1):
        super().__init__('vel_trajectory_controller')
        
        # Trajectory Parameters
        self.start_pos = start_pos
        self.start_rot = [-np.pi/2, 0]
        self.start_pose = [*self.start_pos, *self.start_rot]
        self.offset     = offset
        self.axis_idx   = 2 # z composent in fw_kin
        
        # Controller Parameters
        self.gains = [gain_pos] * 3 + [gain_rot] * 2
        self.max_cart_vel  = max_cart_vel 
        self.max_joint_vel = max_joint_vel
        
        # Robot Kinematics
        self.robot = RobotKinematics()
        self.joint_low_bounds  = np.array(self.robot.joint_bounds)[:, 0]
        self.joint_high_bounds = np.array(self.robot.joint_bounds)[:, 1]

        # Running States :  
        # Move to Start Position with Elbow Up -> Start Proper Velocity trajectory control
        self.direction = 1
        self.current_q = None
        self.elbow_up_reached = False  
        self.target_pose = self.start_pose.copy()

        self.get_logger().info("Velocity Trajectory controller started. ")

        # Prepare Controler to move to start Position
        if not self.check_within_workspace():
            raise RuntimeError("Invalid Workspace Configuration")
        
        # ROS Interface
        self._joint_cmd_pub = self.create_publisher(JointTrajectory, 'joint_cmds', 10)
        self._sub = self.create_subscription(JointState, 'joint_states', self._on_joint_states, qos_profile_sensor_data)
        self.create_timer(0.04, self.control_loop)

    def check_within_workspace(self):
        """Ensures Robot is able to perform desired trajectory"""
        def get_offset_ik(dist):
            p_up, p_down = self.start_pose.copy(), self.start_pose.copy()
            p_up[self.axis_idx]   += dist
            p_down[self.axis_idx] -= dist
            return self.robot.inverse_kinematics(p_up), self.robot.inverse_kinematics(p_down)
    
        self.get_logger().info("Checking start target for reachability...")

        # Check Start Pose is inside recheable workspace
        start_confs = self.robot.inverse_kinematics(self.start_pose)
        if not start_confs:
            self.get_logger().error("Start Pose unreachable!")
            return False
        
        self.get_logger().info("Start Pose recheable! Checking target offset poses recheability...")
        
        # If possible, choose an Elbow Up joint configuration : Prevent floor collision
        self.start_joint_conf = self.pick_elbow_up_conf(start_confs)

        # Check Offset Targets are inside recheable workspace
        conf_up, conf_down = get_offset_ik(self.offset)
        
        # Shrink offset until both ends are reachable     
        if not (conf_up and conf_down):
            self.get_logger().info(f"Offset too large. Computing minimum feasible offset...")

        while  not (conf_up and conf_down) and self.offset > 0:
            self.offset = round(self.offset - 0.02, 3)
            conf_up, conf_down = get_offset_ik(self.offset)
        self.offset = round(self.offset - 0.02, 3)
        self.get_logger().info(f"Target Offset updated: {self.offset}")

        # Update final poses for the controller to use
        self.offset_pose_up = self.start_pose.copy()
        self.offset_pose_up[self.axis_idx] += self.offset
        self.offset_pose_down = self.start_pose.copy()
        self.offset_pose_down[self.axis_idx] -= self.offset
        self.get_logger().info(f"Start Pose:       {self.start_pose}")
        self.get_logger().info(f"Offset Up Pose:   {self.offset_pose_up}")
        self.get_logger().info(f"Offset Down Pose: {self.offset_pose_down}")

        self.get_logger().info("Starting Joint Position Control to move to start position...")
        return True
    
    def pick_elbow_up_conf(self, solutions):
        """ Return the most elbow-up, well-conditioned solution from a list of IK solutions """
        def score(q):
            J = self.robot.jacobian(q)
            sigma_min = np.linalg.svd(J, compute_uv=False)[-1]
            return (q[1] + q[2] + q[3]) - sigma_min * 5  # prefer elbow-up + good conditioning
        return min(solutions, key=score)

    def control_loop(self):
        if self.current_q is None:
            self.get_logger().warn('Waiting for joint_states — check QoS or simulator is running.', throttle_duration_sec=2.0)
            return

        # Phase 1: Joint Velocity Control -> Move to Elbow Up Start Position
        if not self.elbow_up_reached:
            joint_error = self.start_joint_conf - self.current_q
            joint_error_dis = np.linalg.norm(joint_error)

            if joint_error_dis < 0.05:
                self.elbow_up_reached = True
                self.target_pose = self.offset_pose_up
                self.get_logger().info('Elbow-up start configuration reached. Switching to Cartesian Velocity control...')
                return

            joint_vel = np.clip(self.gains * joint_error, -self.max_joint_vel, self.max_joint_vel)
            joint_vel, limited = self.robot.limit_velocities_at_bounds(self.current_q, joint_vel)
            if limited:
                self.get_logger().warn("Joint limit reached — velocity component zeroed")
            self._publish_velocity(joint_vel)
            return
    
        # Phase 2: Cartesian Velocity Control
        car_pose  = self.robot.forward_kinematics(self.current_q)[0:len(self.target_pose)]
        car_error = self.target_pose - car_pose

        # Update Target Pose
        if abs(car_error[self.axis_idx]) < 0.01:  
            self.direction *=-1
            if self.direction == 1:
                self.target_pose = self.offset_pose_up
            else:
                self.target_pose = self.offset_pose_down
            return

        # Build Cartesian Velocity Input
        car_vel = np.zeros(6)
        car_vel[:len(self.target_pose)] = np.array(self.gains) * car_error
        car_vel[self.axis_idx] = self.direction * self.max_cart_vel

        # Built Joint Velocity Input
        jac_inv, is_singular = self.robot.jacobian_inverse(self.current_q)
        if is_singular:
            self.get_logger().info(f"Close to singularity ...")
        
        joint_vel = jac_inv @ car_vel
        joint_vel = np.clip(joint_vel, -self.max_joint_vel, self.max_joint_vel)
        joint_vel, limited = self.robot.limit_velocities_at_bounds(self.current_q, joint_vel)
        if limited:
            self.get_logger().warn("Joint limit reached — velocity component zeroed")
        
        # Publish joint velocity
        self._publish_velocity(joint_vel)
        
    def _publish_velocity(self, q_dot):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.robot.joint_names
        p = JointTrajectoryPoint()
        p.velocities = list(q_dot) + [0.0]  # trailing 0.0 is gripper (joint 6), kept stationary
        msg.points = [p]
        self._joint_cmd_pub.publish(msg)

    def _publish_position(self, q):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.robot.joint_names
        p = JointTrajectoryPoint()
        p.positions = list(q) + [0.0]  # trailing 0.0 is gripper (joint 6), kept stationary
        msg.points = [p]
        self._joint_cmd_pub.publish(msg)

    def _on_joint_states(self, msg: JointState):
        self.current_q = np.array(msg.position[:5])


def main(args=None):
    rclpy.init(args=args)
    # Start at 20cm height, try to oscillate 15cm up/down
    node = VelTrajectoryController(start_pos=[0.2, 0.2, 0.2], offset=0.15)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()