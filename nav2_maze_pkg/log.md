cbp nav2_maze_pkg && roseloq
cbp fusionbot_gazebo && roseloq
cbp custom_interfaces && roseloq
cbp custom_nav2_bt_navigator && roseloq

source /usr/share/gazebo/setup.sh

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

goal action 프로그램 수정

# terminal 1
ros2 launch nav2_maze_pkg maze_gazebo.launch.py
# terminal 2
ros2 launch nav2_maze_pkg custom_bringup_launch.py open_rviz:=true
# terminal 3
ros2 run neuronbot2_path_planning nav_to_pose_action_client.py
Distance from Goal: 1.0264804363250732
Distance from Goal: 1.0264804363250732
Distance from Goal: 1.0265885591506958
Distance from Goal: 1.0265885591506958
Distance from Goal: 1.0265885591506958
Distance from Goal: 1.0266036987304688
Distance from Goal: 1.0266036987304688
...


waypoint 프로그램 만들어 보자

# terminal 1
ros2 launch nav2_maze_pkg maze_gazebo.launch.py
# terminal 2
ros2 launch nav2_maze_pkg custom_bringup_launch.py open_rviz:=true
# terminal 3
ros2 run neuronbot2_path_planning waypoint_follower_demo.py

$ ros2 action list
/FollowWaypoints
/NavigateToPose
/back_up
/compute_path_to_pose
/follow_path
/spin
/wait

/FollowWaypoints

nav2_msgs/action/FollowWaypoints

$ ros2 interface show nav2_msgs/action/FollowWaypoints
#goal definition
geometry_msgs/PoseStamped[] poses
---
#result definition
int32[] missed_waypoints
---
#feedback
uint32 current_waypoint

이건 main이랑 같다 ㅠㅠ
사용 가능! but 그냥 다시 짜자 ㅎㅎ

Obstacle Avoidance => meme world에 actor 추가?
https://navigation.ros.org/plugin_tutorials/docs/writing_new_nav2controller_plugin.html