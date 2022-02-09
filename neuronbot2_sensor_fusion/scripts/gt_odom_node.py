#!/usr/bin/env python3
#
# Copyright 2022 RoadBalance Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from gazebo_msgs.srv import GetEntityState
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock

class EntityStateClient(Node):

    def __init__(self, model_name):
        super().__init__('reset_model_client')

        self._entity_state_client = self.create_client(
            GetEntityState, 'get_entity_state'
        )

        while not self._entity_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(' [gazebo/get_entity_state] service not available, waiting again...')

        self._entity_state_req = GetEntityState.Request()
        self._entity_state_req.name = model_name

        self.get_logger().info('==== Entity State Service Client Ready ====')

    def send_request(self):
        
        future = self._entity_state_client.call_async(self._entity_state_req)
        # self.get_logger().info('=== Request Sended ===')
        
        return future

class OdomUtilNode(Node):

    def __init__(self):
        super().__init__('odometry_util_node')

        self.declare_parameter('model_name', 'neuronbot2')
        self._model_name = self.get_parameter('model_name').value

        self._entity_state_client = EntityStateClient(self._model_name)

        self._gt_odom_publisher = self.create_publisher(
            Odometry, 'gt_odom', 1
        )

        self._sim_clock_subscriber = self.create_subscription(
            Clock, 'clock', self.clock_callback, 10
        )

        self._timer = self.create_timer(0.1, self.timer_callback)
        self._odom_msg = Odometry()
        
        self._odom_msg.child_frame_id = 'base_footprint'
        self._odom_msg.header.frame_id = 'odom'

        self._sim_time = Clock()

    def clock_callback(self, msg):

        self._sim_time.clock.sec = msg.clock.sec
        self._sim_time.clock.nanosec = msg.clock.nanosec

    def timer_callback(self):

        state_client_future = self._entity_state_client.send_request()
        rclpy.spin_until_future_complete(self._entity_state_client, state_client_future)

        if state_client_future.done():
            try:
                state_response = state_client_future.result()
            except Exception:
                raise RuntimeError(
                    'exception while calling entity state service: %r' % state_client_future.exception()
                )
            else:
                self._odom_msg.header.stamp.sec = self._sim_time.clock.sec
                self._odom_msg.header.stamp.nanosec = self._sim_time.clock.nanosec

                self._odom_msg.pose.pose = state_response.state.pose
                self._odom_msg.twist.twist = state_response.state.twist
            finally:
                # self.get_logger().warn('==== Entity Client Execution Done ====')
                pass

        # self.get_logger().info(f"Got ground truth pose & twist")

        self._gt_odom_publisher.publish(self._odom_msg)


def main(args=None):
    rclpy.init(args=args)

    odom_util_node = OdomUtilNode()

    rclpy.spin(odom_util_node)

    odom_util_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()