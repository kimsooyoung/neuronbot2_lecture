import socket 

#UDP Client 
if __name__ == '__main__': 
    client_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #UDP Socket 
    Host = 'localhost' #통신할 대상의 IP 주소 
    Port = 9000 #통신할 대상의 Port 주소 
    
    while True: 
        print('===================================================================') 
        print('=== Type Robot Cmd Vel {linear_x;angular_z} - Example : 0.5;1.0 ===') 
        print('===================================================================')
        print('') 
        print('Client >> ', end='') 
        send_data = bytes(input().encode()) #사용자 입력 
        client_sock.sendto(send_data, (Host, Port)) #Client -> Server 데이터 송신 
        recv_data, addr = client_sock.recvfrom(1024)#Server -> Client 데이터 수신 
        print('Response from Server >> ' + str(recv_data.decode()))
