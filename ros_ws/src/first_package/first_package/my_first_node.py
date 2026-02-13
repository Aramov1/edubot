#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        self.counter = 1
        super().__init__("first_node")
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.counter = self.counter + 1
        self.get_logger().info(f"Time: {self.counter}")


def main(args=None):
    rclpy.init(args=args)
    first_node = MyNode()
    rclpy.spin(first_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()