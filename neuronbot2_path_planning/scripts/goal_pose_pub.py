#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped

class GoalPosePublisher(Node):

    def __init__(self):
        super().__init__('goal_pose_pub_node')

        self._goal_pose_pub = self.create_publisher(
            PoseStamped, 'goal_pose', 1
        )

        self._msg = PoseStamped()

    def goal_publish(self):

        while True:
            try:
                pos_x = float(input("X Pos : "))
                pos_y = float(input("Y Pos : "))
                quat_w = float(input("W Quat : "))
            except Exception as e:
                print(e)
            finally:
                self._msg.header.frame_id = 'map'

                self._msg.pose.position.x = pos_x
                self._msg.pose.position.y = pos_y
                self._msg.pose.orientation.w = quat_w

                self.get_logger().info(f"""
                    Publishing  Goal Position  \nX= {pos_x}\nY= {pos_y}\nW= {quat_w}
                """)

                self._goal_pose_pub.publish(self._msg)
                break

def main(args=None):

    rclpy.init(args=args)
    goal_publisher = GoalPosePublisher()
    
    goal_publisher.goal_publish()
    
    goal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()