#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from rosgraph_msgs.msg import Clock

class InitialPosePublisher(Node):

    def __init__(self):
        super().__init__('initial_pose_pub_node')

        self._publisher = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 1
        )
        
        self._sim_clock_subscriber = self.create_subscription(
            Clock, 'clock', self.clock_callback, 10
        )

        self.msg = PoseWithCovarianceStamped()

    def clock_callback(self, msg):

        print(msg)
    
        self.msg.header.frame_id = 'map'
        self.msg.header.stamp.sec = msg.clock.sec
        self.msg.header.stamp.nanosec = msg.clock.nanosec

        self.msg.pose.pose.position.x = float(input('> X position : '))
        self.msg.pose.pose.position.y = float(input('> Y position : '))
        self.msg.pose.pose.orientation.w = float(input('> Yaw Orientation : '))

        self.msg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, \
            0.0, 0.0, 0.25, 0.0, 0.0, \
            0.0, 0.0, 0.0, 0.0, 0.0, \
            0.0, 0.0, 0.0, 0.0, 0.0, \
            0.0, 0.0, 0.0, 0.0, 0.0, \
            0.0, 0.0, 0.0, 0.0, 0.0, \
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
        
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