# fusion bot

gazebo spawn, urdf만으로 하기

skidbot은 잘 뜨는데... fusionbot은 안뜬다.
이 둘의 차이가 뭘까...
        
```
<mesh filename="package://fusionbot_description/meshes/base_link.stl" scale="0.001 0.001 0.001"/>
```

이런 package 때문일까??


그런데, ROS1은 xacro로도 가제보 잘 나온다.
robot_state_publisher 방식을 바꿔보자.

xacro 사용으로 변경

https://www.theconstructsim.com/how-to-use-xacro-in-ros2-gazebo/

```
[ERROR] [robot_state_publisher-4]: process has died [pid 30450, exit code 255, cmd '/opt/ros/eloquent/lib/robot_state_publisher/robot_state_publisher --ros-args --params-file /tmp/launch_params_xxfiap40'].
```

robot_state_publish에서 문제가 생기네...

eloquent에서는 이렇게 arguments에 파일 경로를 바로 넣어주면 됨.

```
    robot_state_publisher = Node(
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        node_name='robot_state_publisher',
        output='screen',
        # parameters=[urdf_file]
        arguments=[urdf_file]
    )
```

그런데... 이제 spawn이 안되는 문제가 생긴다...

spawn을 코딩으로 해보자 ㅠㅠ




```
[ekf_node-1] Warning: Invalid frame ID "imu_link" passed to canTransform argument source_frame - frame does not exist
```

=> robot_localization 틀었는데 자꾸 이런 에러 난다.
=> ekf.yaml에 이렇게 use_sim_time을 주면 됨
```
### ekf config file ###
ekf_filter_node:
    ros__parameters:
        use_sim_time: true

```

현재 tf odom은 바퀴에서 나오는 것이다.
ekf.yaml에서 publish_tf: true로 하면 되긴 하는데 rviz에서 자꾸 순간이동하는 현상 발생하기 때문에 하나만 보이게 false로 하였음.

laser scan rviz에서 보려면 qos option Best Effort로 해야 한다.


# nav2 로봇 중심이 goal에 오게 하기

edison 아저씨 매개변수를 한단락씩 넣어보기

1. amcl => no
2. 