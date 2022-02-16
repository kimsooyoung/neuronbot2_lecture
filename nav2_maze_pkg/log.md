cbp nav2_maze_pkg && roseloq
cbp custom_interfaces && roseloq
cbp custom_nav2_bt_navigator && roseloq

# terminal 1
ros2 launch nav2_maze_pkg maze_gazebo.launch.py
# terminal 2
ros2 launch nav2_maze_pkg bringup_launch.py open_rviz:=true
Or
ros2 launch nav2_maze_pkg custom_bringup_launch.py open_rviz:=true


Or

# terminal 2
ros2 launch nav2_maze_pkg localization_launch.py use_sim_time:=true
# terminal 3
ros2 launch nav2_maze_pkg navigation_launch.py use_sim_time:=true
Or
ros2 launch nav2_maze_pkg custom_navigation_launch.py use_sim_time:=true
# terminal 4
ros2 launch nav2_maze_pkg rviz_view_launch.py use_sim_time:=true