import rclpy
from rclpy.node import Node

import tf2_ros
from scipy.spatial.transform import Rotation as R

class ReadEECart(Node):
    def __init__(self, verbose = False):
        super().__init__('read_ee_cart')
        self.verbose = verbose

        # TF2 Setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_found = False

        # Timer to poll the transform at 2Hz
        self.create_timer(0.5, self.report_pose)
        self.get_logger().info("End-Effector Reader Started. Waiting for TF...")

    def get_ee_pose(self):
        """Looks up the latest transform and converts it to [x, y, z, p, r, y]."""
        try:
            # Look up transform from world to gripper
            t = self.tf_buffer.lookup_transform('world', 'gripper_center', rclpy.time.Time())
            
            # Position
            pos = t.transform.translation
            
            # Orientation: Convert Quaternion to Euler using Scipy (much cleaner)
            q = t.transform.rotation
            quat = [q.x, q.y, q.z, q.w]
            # 'xyz' (lowercase) = Extrinsic XYZ: rotations about fixed axes,
            # matching the FK formula R = Rz(rot_z) * Ry(rot_y) * Rx(rot_x).
            euler = R.from_quat(quat).as_euler('xyz', degrees=False)
            
            # Return combined list: [x, y, z, rot_x, rot_y, rot_z]
            return [pos.x, pos.y, pos.z, euler[0], euler[1], euler[2]]

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None

    def report_pose(self):
        
        # Print the pose to the terminal
        pose = self.get_ee_pose()
        
        if pose:
            # If first connection, update connection flag
            if not self.tf_found:
                self.get_logger().info("TF Connection Established!")
                self.tf_found = True    
            
            if self.verbose:
                x, y, z, r, p, y = pose
                self.get_logger().info(
                    f"  POS (m): x={x:6.3f} y={y:6.3f} z={z:6.3f}"
                    f"  ROT (rad): rot_x={r:6.3f} rot_y={p:6.3f} rot_z={y:6.3f}\n"
                )
        else:
            self.get_logger().warn("Waiting for TF frame ... ", throttle_duration_sec=5.0)

def main():
    rclpy.init()
    node = ReadEECart(verbose=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()