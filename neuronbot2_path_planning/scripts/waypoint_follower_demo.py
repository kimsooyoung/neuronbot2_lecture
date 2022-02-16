#!/usr/bin/env python3

from nav2_msgs.action import FollowWaypoints
from custom_interfaces.msg import GoalFeedback
from geometry_msgs.msg import PoseStamped

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

class WaypointActionClient(Node):

    def __init__(self):
        super().__init__('Waypoint_Action_Client')
        
        self._action_client = ActionClient(
            self, FollowWaypoints, 'FollowWaypoints'
        )

        self._goal_pose = FollowWaypoints.Goal()

        self._feedback_cnt = 0

    def send_goal(self):

        goal_poses = []
        goal_pose1 = PoseStamped()
        goal_pose1.header.frame_id = 'map'
        goal_pose1.header.stamp = self.get_clock().now().to_msg()
        goal_pose1.pose.position.x = 0.017
        goal_pose1.pose.position.y = 3.29
        goal_pose1.pose.orientation.w = 1.0
        goal_poses.append(goal_pose1)

        goal_pose2 = PoseStamped()
        goal_pose2.header.frame_id = 'map'
        goal_pose2.header.stamp = self.get_clock().now().to_msg()
        goal_pose2.pose.position.x = 2.22
        goal_pose2.pose.position.y = 2.99
        goal_pose2.pose.orientation.z = 0.707
        goal_pose2.pose.orientation.w = 0.707
        goal_poses.append(goal_pose2)

        goal_pose3 = PoseStamped()
        goal_pose3.header.frame_id = 'map'
        goal_pose3.header.stamp = self.get_clock().now().to_msg()
        goal_pose3.pose.position.x = 2.00
        goal_pose3.pose.position.y = -0.39
        goal_pose3.pose.orientation.z = 1.0
        goal_poses.append(goal_pose3)

        goal_pose4 = PoseStamped()
        goal_pose4.header.frame_id = 'map'
        goal_pose4.header.stamp = self.get_clock().now().to_msg()
        goal_pose4.pose.position.x = 3.51
        goal_pose4.pose.position.y = -0.21
        goal_pose4.pose.orientation.z = -0.707
        goal_pose4.pose.orientation.w = 0.707
        goal_poses.append(goal_pose4)

        goal_pose5 = PoseStamped()
        goal_pose5.header.frame_id = 'map'
        goal_pose5.header.stamp = self.get_clock().now().to_msg()
        goal_pose5.pose.position.x = 3.44
        goal_pose5.pose.position.y = 2.99
        goal_pose5.pose.orientation.w = 1.0
        goal_poses.append(goal_pose5)

        self._goal_pose.poses = goal_poses

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            self._goal_pose, feedback_callback= self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):

        self._feedback_cnt += 1

        feedback = feedback_msg.feedback

        if self._feedback_cnt % 10 == 0:
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

    action_client = WaypointActionClient()
    action_client.send_goal()

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
