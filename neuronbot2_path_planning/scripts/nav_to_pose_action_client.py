#!/usr/bin/env python3

from nav2_msgs.action import NavigateToPose
from custom_interfaces.msg import GoalFeedback

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

class NavToPoseActionClient(Node):

    def __init__(self):
        super().__init__('Nav_To_Pose_Action_Client')
        
        self._action_client = ActionClient(
            self, NavigateToPose, 'NavigateToPose'
        )

        self._topic_sub = self.create_subscription(
            GoalFeedback,
            'goal_status',
            self.goal_status_callback,
            10)

        self._goal_pose = NavigateToPose.Goal()

    def goal_status_callback(self, msg):

        print(f"Distance from Goal: {msg.distance_remaining}")

    def send_goal(self):

        while True:
            try:
                pos_x = float(input("X Pos : "))
                pos_y = float(input("Y Pos : "))
                quat_w = float(input("W Quat : "))
            except Exception as e:
                print(e)
            finally:
                self._goal_pose.pose.header.frame_id = 'map'

                self._goal_pose.pose.pose.position.x = pos_x
                self._goal_pose.pose.pose.position.y = pos_y
                self._goal_pose.pose.pose.position.z = quat_w

                self.get_logger().info(f"""
                    Publishing  Goal Position  \nX= {pos_x}\nY= {pos_y}\nW= {quat_w}
                """)

                break

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            self._goal_pose, feedback_callback= self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        self.get_logger().info(f"Feedback from Srv: {str(feedback)}")

    def goal_response_callback(self, future):
        
        goal_handle = future.result()

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):

        result = future.result().result
        self.get_logger().info(f"Feedback from Srv: {str(result)}")

def main(args=None):
    rclpy.init(args=args)

    action_client = NavToPoseActionClient()
    action_client.send_goal()

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
