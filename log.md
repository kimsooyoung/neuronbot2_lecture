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

# waypoint 

```
waypoint_follower:
  ros__parameters:
    loop_rate: 1
    stop_on_failure: false
```

ros2 pkg create --build-type ament_cmake fusionbot_description
ros2 pkg create --build-type ament_cmake fusionbot_gazebo

여러 world
straight line follower plugin
nav2 bonus


first_test

1k3t7g547K

 ros2 launch neuronbot2_path_planning bringup_launch.py open_rviz:=true
ros2 launch neuronbot2_nav bringup_launch.py open_rviz:=true
ros2 launch neuronbot2_gazebo neuronbot2_world.launch.py world_model:=mememan_world.model

straight는 되는데, waypoint가 안되네...

nav2 잘 도착하게 만들기

git clone 
checkout => foxy-devel
[model_paths]
filenames=/home/kimsooyoung/neuronbot2_eloquent_ws/src/neuronbot2/neuronbot2_lecture:/mnt/c/Users/tge13/Documents/Dev/aws-robomaker-small-warehouse-world/models

cd ~/neuronbot2_eloquent_ws/src
git clone https://github.com/SteveMacenski/nav2_rosdevday_2021.git

 cbp nav2_rosdevday_2021 => ok

cd ~/neuronbot2_eloquent_ws/src
git clone https://github.com/neobotix/neo_simulation2.git
cd ../
cbp neo_simulation2 => ok!

cd ~/neuronbot2_eloquent_ws/src
git clone -b foxy-devel https://github.com/aws-robotics/aws-robomaker-small-warehouse-world.git
cbp aws_robomaker_small_warehouse_world && roseloq

launch 파일 수정 => path 2개

spawn entity 안됨 ㅋㅋㅋ
=> 심지어 neo_simlation2에서는 urdf를 사용중이다. 

=> neuronbot2로 바꿔서 싹다 해보던지 해야 함!!
=> nav2 최적화 문제로 귀결된다.

nav2 잘됨 => 왜 그럴까?

1. amcl 문단 통째로 가져옴 => ok
2. min_vel_x => 이게 음수이니까 도착을 못함 => 0.0으로 해야 도착함
3. max_vel_theta => 문제 없음 1.0에서 0.8로 유지
4. acc_lim_x => 상관 있다. 0.5에서 2.5로 키워줌
5. acc_lim_theta / decel_lim_theta => 큰 영향은 없지만 크면 좋을 듯 일단 기본값 +-0.8 유지
	=> 0.8 유지한 채로 직선 경로로 수정하면?
6. vy_samples => 5로 올려야 함


nav2_utils foxy => eloquent
```
This issue still exist when compile the example code from https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Cpp-Service-And-Client.html using eloquent. It can be solved by adding namespace. That means replace the rclcpp::FutureReturnCode::SUCCESS with rclcpp::executor::FutureReturnCode::SUCCESS.
```

# custom nav2 pkgs

종속성

```
nav2_util => custom_nav2_behavior_tree => custom_nav2_bt_navigator
```

무조건 nav2_util  => custom_nav2_util 로 바꾸면 안된다. 
namespace 이름 자체가 nav2_util 이기 때문에 일부는 유지해줘야 함


custom_nav2_util => geometry_utils.hpp
```
namespace nav2_util
{
namespace geometry_utils
{
```
이렇게

에러
```
CMake Warning at CMakeLists.txt:53 (add_library):
  Cannot generate a safe runtime search path for target
  custom_bt_navigator_core because there is a cycle in the constraint graph:
```
nav2_msgs, custom_nav2_msgs 이렇게 두개 있어서 그럼

```
[bt_navigator-7] [ERROR] [bt_navigator]: Couldn't open input XML file:
```

custom_bt_navigator 안에 있는 파일로 대체, custom launch file 생성

```
[custom_bt_navigator-7] [ERROR] [bt_navigator_rclcpp_node.rclcpp_action]: Error in destruction of rcl action client handle: goal client is invalid, at /tmp/binarydeb/ros-eloquent-rcl-action-0.8.5/src/rcl_action/action_client.c:448
[custom_bt_navigator-7] [ERROR] []: Caught exception in callback for transition 10
[custom_bt_navigator-7] [ERROR] []: Original error: Cannot load library
[custom_bt_navigator-7] [WARN] []: Error occurred while doing error handling.
[custom_bt_navigator-7] [FATAL] [bt_navigator]: Lifecycle node entered error state
```

# nav2 rviz plugin

```
Unknown CMake command "ament_export_targets".
Navigate to the CMakeLists.txt line 144 and change ament_export_targets to ament_export_interfaces.
```

# rviz pluigin

```
[rviz2-10] [ERROR] [rviz2]: PluginlibFactory: The plugin for class 'custom_nav2_rviz_plugins/Navigation 2' failed to load. Error: According to the loaded plugin descriptions the class custom_nav2_rviz_plugins/Navigation 2 with base class type rviz_common::Panel does not exist. Declared types are  nav2_rviz_plugins/Navigation 2
```

# custom bt navigator 중

```
[bt_navigator-7] [WARN] [LifecyclePublisher]: Trying to publish message on the topic '/goal_status', but the publisher is not activated
```

# custom nav2 pkgs

종속성

```
nav2_util => custom_nav2_behavior_tree => custom_nav2_bt_navigator
```

무조건 nav2_util  => custom_nav2_util 로 바꾸면 안된다. 
namespace 이름 자체가 nav2_util 이기 때문에 일부는 유지해줘야 함


custom_nav2_util => geometry_utils.hpp
```
namespace nav2_util
{
namespace geometry_utils
{
```
이렇게

에러
```
CMake Warning at CMakeLists.txt:53 (add_library):
  Cannot generate a safe runtime search path for target
  custom_bt_navigator_core because there is a cycle in the constraint graph:
```
nav2_msgs, custom_nav2_msgs 이렇게 두개 있어서 그럼

```
[bt_navigator-7] [ERROR] [bt_navigator]: Couldn't open input XML file:
```

```
[gzclient -2] gzclient: /usr/include/boost/smart_ptr/shared_ptr.hpp:734: typename boost::detail::sp_member_access<T>::type boost::shared_ptr<T>::operator->() const [with T = gazebo::rendering::Camera; typename boost::detail::sp_member_access<T>::type = gazebo::rendering::Camera*]: Assertion `px != 0' failed.
```

source /usr/share/gazebo/setup.sh