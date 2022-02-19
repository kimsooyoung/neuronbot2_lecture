cbp neuronbot2_obstacle_avoidance && roseloq

```
ros2 launch neuronbot2_gazebo neuronbot2_world.launch.py world_model:=mememan_world.model

ros2 launch neuronbot2_obstacle_avoidance bringup_launch.py open_rviz:=true

```