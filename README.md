# neuronbot2_lecture



```
ros2 run tf2_tools view_frames
```

```
cbp custom_interfaces
cbp custom_nav2_bt_navigator
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

# Custom bt navigator

```
ros2 launch neuronbot2_gazebo neuronbot2_world.launch.py world_model:=mememan_world.model
ros2 launch neuronbot2_path_planning custom_bringup_launch.py open_rviz:=true
ros2 topic echo /goal_status
```


# Conflict Test

