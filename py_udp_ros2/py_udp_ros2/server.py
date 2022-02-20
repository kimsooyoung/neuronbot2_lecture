# !/usr/bin/env/ python3

import rclpy
import socket 
from rclpy.node import Node
from geometry_msgs.msg import Twist


class UDPCmdVelPublisher(Node):

    def __init__(self):
        super().__init__('udp_cmd_vel_pub_node')

        self.publisher = self.create_publisher(Twist, 'skidbot/cmd_vel', 10)
        
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.publish_callback)
        
        self.get_logger().info(
            'Node publisher creation done. \n'
        )

        self.pub_msg = Twist()

    def get_cmd_vel(self, linear_x, angular_z):
        self.pub_msg.linear.x = float(linear_x)
        self.pub_msg.angular.z = float(angular_z)

    def pub_cmd_vel(self):
        self.publisher.publish(self.pub_msg)


class UDPSocket(object):

    def __init__(self):
        HOST = '' #수신 받을 모든 IP를 의미 
        PORT = 9000 #수신받을 Port 
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #UDP Socket 
        self.server_sock.bind((HOST, PORT)) #소켓에 수신받을 IP주소와 PORT를 설정


def main(args=None):
    rclpy.init(args=args)

    cmd_vel_publisher = UDPCmdVelPublisher()

    HOST = '' #수신 받을 모든 IP를 의미 
    PORT = 9000 #수신받을 Port 
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #UDP Socket 
    server_sock.bind((HOST, PORT)) #소켓에 수신받을 IP주소와 PORT를 설정 

    while True:
        data, addr = server_sock.recvfrom(1024) #Client -> Server 데이터 수신 
        str_data = data.decode('utf-8')

        cmds = str_data.split(';')
        print(f'>> linear_x : {cmds[0]} / angular_z : {cmds[1]}')
        # print('Client >> ' + str_data) 
        server_sock.sendto(data, (addr)) #Server -> Client 데이터 송신

        cmd_vel_publisher.get_cmd_vel(cmds[0], cmds[1])
        cmd_vel_publisher.pub_cmd_vel()

        rclpy.spin(cmd_vel_publisher)

    cmd_vel_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()