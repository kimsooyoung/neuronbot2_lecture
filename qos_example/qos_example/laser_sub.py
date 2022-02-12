#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

class LaserSub(Node):

    def __init__(self):
        super().__init__('laser_sub_node')

        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            durability=QoSDurabilityPolicy.VOLATILE)

        # self._sub = self.create_subscription(
        #     LaserScan, 'scan', self.sub_callback, QOS_RKL10V
        # )

        self._sub = self.create_subscription(
            LaserScan, 'scan', self.sub_callback, 10
        )

        self.get_logger().info(f"LaserSub with QoS Created")

    def sub_callback(self, msg):
        self.get_logger().info(f"Sub Msg : {msg.ranges[180]}")

# The following is just to start the node
def main(args=None):
    
    rclpy.init(args=args)

    node = LaserSub()
    rclpy.spin(node)
    node.destroy_node()
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()