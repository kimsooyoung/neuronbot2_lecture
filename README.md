# neuronbot2_lecture



```
ros2 run tf2_tools view_frames
```

```
sudo apt install ros-eloquent-launch-testing-ament-cmake \
                ros-eloquent-geographic-msgs \
                ros-eloquent-diagnostic-updater \
                ros-eloquent-diagnostic-msgs -y

cd ~/neuronbot2_eloquent_ws/src
git clone -b eloquent-devel https://github.com/Adlink-ROS/robot_localization.git
cd ~/neuronbot2_eloquent_ws
cba
```

```
ros2 launch neuronbot2_gazebo neuronbot2_world.launch.py
ros2 launch neuronbot2_sensor_fusion robot_localization.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

# AMCL
```
ros2 launch neuronbot2_gazebo neuronbot2_world.launch.py world_model:=mememan_world.model

ros2 launch neuronbot2_amcl amcl.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

# nav2
```
ros2 launch neuronbot2_gazebo neuronbot2_world.launch.py world_model:=mememan_world.model
ros2 launch neuronbot2_path_planning bringup_launch.py open_rviz:=true

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
