#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class TestNode(Node):

    def __init__(self):
        super().__init__('test_py_node')

        self.get_logger().info("test")


def main(args=None):
    rclpy.init(args=args)

    node = TestNode()
    rclpy.spin(node)
    node.destroy_node()
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()