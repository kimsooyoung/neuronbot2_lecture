
```
ros2 launch neuronbot2_gazebo neuronbot2_world.launch.py world_model:=mememan_world.model
ros2 launch neuronbot2_amcl amcl.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```