cbp nav2_cafe_pkg && roseloq
ros2 launch nav2_cafe_pkg caffee_world.launch.py
ros2 launch nav2_cafe_pkg no_actor_caffee_world.launch.py

바닥 높이랑 actor들은 무관하다.
actor들은 바닥 뚫어버림

바닥 마찰때문에 로봇이 잘 못가니 cafe 자체를 조금 내려주었다.

# terminal 1
ros2 launch nav2_cafe_pkg caffee_world.launch.py
# terminal 2
ros2 launch nav2_cafe_pkg bringup_launch.py open_rviz:=true
