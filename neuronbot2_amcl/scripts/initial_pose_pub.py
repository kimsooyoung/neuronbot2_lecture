#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped

class InitialPosePublisher(Node):

    def __init__(self):
        super().__init__('initial_pose_pub_node')

        self._publisher = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 1
        )
        
        timer_period = 0.5  # seconds

        self.timer_ = self.create_timer(
            timer_period, self.timer_callback
        )
        
        self.msg = PoseWithCovarianceStamped()

    def timer_callback(self):

        self.msg.header.frame_id = 'map'
        self.msg.header.stamp = self.get_clock().now().to_msg()

        self.msg.pose.pose.position.x = float(input('> X position : '))
        self.msg.pose.pose.position.y = float(input('> Y position : '))
        self.msg.pose.pose.orientation.w = float(input('> Yaw Orientation : '))
        
        self.get_logger().info(f"""Publishing Initial Position
            X: {self.msg.pose.pose.position.x}
            Y: {self.msg.pose.pose.position.y}
            W: {self.msg.pose.pose.orientation.w}"""
        )
        
        self._publisher.publish(self.msg)

def main(args=None):

    rclpy.init(args=args)
    initial_pose_publisher = InitialPosePublisher()
    
    rclpy.spin_once(initial_pose_publisher)
    
    initial_pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()