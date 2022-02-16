cbp custom_interfaces && roseloq
cbp custom_nav2_bt_navigator && roseloq
cbp neuronbot2_path_planning && roseloq

cbp nav2_text_pkg && roseloqZ

ros2 launch neuronbot2_path_planning maze_gazebo.launch.py
ros2 launch neuronbot2_path_planning maze_bringup_launch.py

ros2 launch neuronbot2_gazebo neuronbot2_world.launch.py world_model:=mememan_world.model
ros2 launch neuronbot2_nav bringup_launch.py open_rviz:=true
ros2 launch nav2_text_pkg bringup_launch.py open_rviz:=true

ros2 launch neuronbot2_path_planning maze_bringup_launch.py

# terminal 1
ros2 launch neuronbot2_path_planning maze_gazebo.launch.py
# terminal 2
ros2 launch neuronbot2_path_planning localization_launch.py use_sim_time:=true
# terminal 3
ros2 launch neuronbot2_path_planning navigation_launch.py use_sim_time:=true
# terminal 4
ros2 launch neuronbot2_path_planning rviz_view_launch.py use_sim_time:=true

```
[bt_navigator-4] [INFO] [bt_navigator]: Begin navigating from current location to (0.04, 2.14)
[bt_navigator-4] [ERROR] [bt_navigator]: Action server failed while executing action callback: "Error at line 21: -> Node not recognized: GoalUpdated"
[bt_navigator-4] [WARN] [bt_navigator]: [NavigateToPose] [ActionServer] Aborting handle.
```