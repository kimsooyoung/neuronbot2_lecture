#!/usr/bin/env python3

import os
import sys
import rclpy
from rclpy.node import Node

import xacro

from gazebo_msgs.srv import SpawnEntity

def main(args= None):
    rclpy.init(args=args)
    node = rclpy.create_node('minimal_client')
    cli = node.create_client(SpawnEntity, "/spawn_entity")

    content = "/home/kimsooyoung/neuronbot2_eloquent_ws/src/neuronbot2/neuronbot2_lecture/fusionbot_description/urdf/fusionbot.urdf"
    content_config = xacro.process_file(content)
    content_xml = content_config.toxml()

    # doc = xacro.parse(open(urdf_file))

    # urdf_file = os.path.join(description_pkg_path, 'urdf', 'skidbot.urdf')
    # doc = xacro.parse(open(urdf_file))
    # xacro.process_doc(doc)
    # robot_description = {'robot_description': doc.toxml()}
    # param = {'use_sim_time': True, 'robot_description': doc.toxml()}


    req = SpawnEntity.Request()
    req.name = ""
    req.xml  = content_xml 
    req.robot_namespace = ""
    req.reference_frame = "world"

    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Service not found")
    
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(f"Result {future.result().success}")
    else:
        node.get_logger().error("Fail")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()

