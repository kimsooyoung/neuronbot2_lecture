#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class TestClass(Node):

    def __init__(self):
        super.__init__('test_node')


def main(args=None):
    rclpy.init(args=args)

    test_class = TestClass()
    rclpy.spin(test_class)

    rclpy.spin_once(test_class)

if __name__ == '__main__':
    main()
