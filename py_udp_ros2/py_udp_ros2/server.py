#!/usr/bin/env/ python3

import rclpy
import socket

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist


class UDPCmdVelPublisher(Node):

    def __init__(self):
        super().__init__('udp_cmd_vel_pub_node')

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_callback)
        
        self.get_logger().info('Node publisher creation done.')

        self.pub_msg = Twist()

    def publish_callback(self):
        self.publisher.publish(self.pub_msg)

    def get_cmd_vel(self, linear_x, angular_z):
        self.pub_msg.linear.x = float(linear_x)
        self.pub_msg.angular.z = float(angular_z)

    def pub_cmd_vel(self):
        self.publisher.publish(self.pub_msg)

class UDPSocket(Node):

    def __init__(self):
        super().__init__('udp_server')

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.server_callback)

        HOST = '' #수신 받을 모든 IP를 의미 
        PORT = 9000 #수신받을 Port 
        self._server_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #UDP Socket 
        self._server_sock.bind((HOST, PORT)) #소켓에 수신받을 IP주소와 PORT를 설정
        
        self._cmds = [0.0, 0.0]

        self.get_logger().info('=== Socket Binding Done ===')
        print('Waiting for Clinet ...\n')

    @property
    def cmds(self):
        return self._cmds

    def server_callback(self):
        data, addr = self._server_sock.recvfrom(1024) #Client -> Server 데이터 수신 
        str_data = data.decode('utf-8')

        self._cmds = str_data.split(';')
        print(f'>> linear_x : {self._cmds[0]} / angular_z : {self._cmds[1]}')

        self._server_sock.sendto(data, (addr)) #Server -> Client 데이터 송신

def main(args=None):
    rclpy.init(args=args)

    cmd_vel_publisher = UDPCmdVelPublisher()
    udp_socket = UDPSocket()

    executor = MultiThreadedExecutor()
    executor.add_node(cmd_vel_publisher)
    executor.add_node(udp_socket)

    while True:
        linear_x, angular_z = udp_socket.cmds
        cmd_vel_publisher.get_cmd_vel(linear_x, angular_z)

        executor.spin_once()

    cmd_vel_publisher.destroy_node()
    udp_socket.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()