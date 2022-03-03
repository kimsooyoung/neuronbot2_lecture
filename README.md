# neuronbot2_lecture

Packages for ROS 2 Autonomous Robot lecture.

## Day 1
---

### Robot Description

* package build

```bash
cd ~/<your-workspace>
colcon build --symlink-install --packages-select neuronbot2_lecture
source ./install/setup.bash
```

* example1 - neuronbot2 description 

```
ros2 launch neuronbot2_lecture neuronbot_description.launch.py
```

<p align="center">
    <img src="./media/neuronbot2_description.gif" height="300">
</p>

* example2 - spot description

<p align="center">
    <img src="./media/spot_description.gif" height="300">
</p>

## Day 2
---

### Gazebo with Robot

* package build

```bash
cd ~/<your-workspace>
colcon build --symlink-install --packages-select fusionbot_gazebo
source ./install/setup.bash

colcon build --symlink-install --packages-select fusionbot_description
source ./install/setup.bash
```

* example1 - Add various gazebo models from osrf github

<p>
    <p align="center">
        <img src="./media/various_model_gazebo.png" height="200">
    </p>
</p>

```bash
cd ~/.gazebo/models
git clone https://github.com/osrf/gazebo_models.git

cd gazebo_models
rm -rf ground_plane
rm -rf sun
mv * ../
```

* example2 - Fusion360 model to Gazebo

<p>
    <p align="center">
        <img src="./media/fusionbot_modeling.png" height="200">
        <img src="./media/fusionbot_gazebo.gif" height="200">
    </p>
</p>

prepare `fusion2urdf` scripts for conversion.

```bash
git clone https://github.com/syuntoku14/fusion2urdf.git 

cd fusion2urdf
cp -r ./URDF_Exporter /mnt/c/Users/<user-name>/AppData/Roaming/Autodesk/Autodesk\ Fusion\ 360/API/Scripts/
```

| referenced from [Jerin Peter](https://www.youtube.com/channel/UCBZJJmnJtioJtXZqPKJ3CfQ)'s video : https://www.youtube.com/watch?v=cQh0gNfb6ro

Play with fusionbot and Lidar sensors

<p>
    <p align="center">
        <img src="./media/fusionbot_gazebo.png" height="200">
    </p>
</p>

```
ros2 launch fusionbot_description fusionbot_gazebo.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

* example3 - Gazebo Building Editor

<p>
    <p align="center">
        <img src="./media/gazebo_building.png" height="150">
        <img src="./media/gazebo_building_with_robot.gif" height="200">
    </p>
</p>

```
ros2 launch fusionbot_gazebo my_world.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

* example4 - Moving Gazebo Objects

<p>
    <p align="center">
        <img src="./media/gazebo_cafe.png" height="200">
    </p>
</p>

```
ros2 launch fusionbot_gazebo caffee_world.launch.py
```

<p>
    <p align="center">
        <img src="./media/moving_box.gif" height="200">
    </p>
</p>

```
ros2 launch fusionbot_gazebo moving_box.launch.py
```

<p>
    <p align="center">
        <img src="./media/walk.gif" height="200">
    </p>
</p>

```
ros2 launch fusionbot_gazebo walk.launch.py
```

<p>
    <p align="center">
        <img src="./media/moon_walk.gif" height="200">
    </p>
</p>

```
ros2 launch fusionbot_gazebo moon_walk.launch.py
```

<p>
    <p align="center">
        <img src="./media/synced_walk.gif" height="200">
    </p>
</p>

```
ros2 launch fusionbot_gazebo synced_actor.launch.py
```

* Amazon [small_warehouse_world](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world)

<p>
    <p align="center">
        <img src="./media/aws_world.png" height="200">
    </p>
</p>

```bash
cd ~/<your-ws>/src/
git clone -b ros2 https://github.com/aws-robotics/aws-robomaker-small-warehouse-world.git
cd ~/<your-ws>

cbp aws_robomaker_small_warehouse_world && roseloq
source /usr/share/gazebo/setup.sh

# launch
ros2 launch fusionbot_gazebo aws_factory.launch.py
```

* Fusionbot in Amazon small warehouse

<p>
    <p align="center">
        <img src="./media/fusionbot_aws.gif" height="200">
    </p>
</p>

```
ros2 launch fusionbot_gazebo fusionbot_aws_factory.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

* MIT [RaceCar world](https://github.com/mit-racecar/racecar_gazebo)

<p>
    <p align="center">
        <img src="./media/racecourse.png" height="200">
    </p>
</p>

```
ros2 launch fusionbot_gazebo racecourse.launch.py
```

### Odometry and Sensor Fusion

* various odometry

Lidar Odom

<p>
    <p align="center">
        <img src="./media/laser_odom.gif" height="200">
        <img src="./media/various_odom.png" height="200">
    </p>
</p>

```bash
# Package build
cbp rf2o_laser_odometry && roseloq && cbp neuronbot2_lecture && roseloq 

# launch in each terminal
ros2 launch neuronbot2_gazebo neuronbot2_world.launch.py world_model:=mememan_world.model
ros2 launch neuronbot2_lecture laser_odom.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

* robot_localization

build packages

```bash
cd ~/<your-ws>/src
git clone -b eloquent-devel https://github.com/Adlink-ROS/robot_localization.git
cd ~/<your-ws>/

colcon build --symlink-install --packages-select robot_localization
# long waiting time
source ./install/setup.bash

colcon build --symlink-install --packages-select neuronbot2_sensor_fusion
source ./install/setup.bash
```

Filtered Odom

<p>
    <p align="center">
        <img src="./media/filtered_odom.gif" height="200">
        <img src="./media/filtered_odom.png" height="100">
    </p>
</p>

```bash
# launch in each terminals
ros2 launch neuronbot2_sensor_fusion neuronbot2_world.launch.py
ros2 launch neuronbot2_sensor_fusion robot_localization.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Comparison with Ground Truth Odom

<p align="center">
    <img src="./media/gt_odom.png" height="200">
</p>

```bash
# launch in each terminals
ros2 launch neuronbot2_sensor_fusion neuronbot2_world.launch.py
ros2 launch neuronbot2_sensor_fusion gt_odom_pub.launch.py
ros2 launch neuronbot2_sensor_fusion robot_localization.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
rqt
```

If there's an error during `gt_odom_pub.launch.py`

```bash
cd ~/neuronbot2_eloquent_ws/src/neuronbot2/neuronbot2_lecture/neuronbot2_sensor_fusion/scripts
chmod +x *

sudo apt install dos2unix
dos2unix gt_odom_node.py

cd ~/neuronbot2_eloquent_ws
cbp neuronbot2_sensor_fusion && roseloq
```

* Real Robot odometry and Sensor Fusion

<p>
    <p align="center">
        <img src="./media/real_neuronbot.png" height="200">
        <img src="./media/real_neuronbot.gif" height="200">
    </p>
</p>

You can replay my log through `rosbag`, which located in rosbag file

* laser_odom_log_1
* laser_odom_log_2
* laser_odom_log_3
* neuronbot2_20220117_03

rosbag replay and visualization with rviz

<p>
    <p align="center">
        <img src="./media/rosbag_odom1.png" height="200">
        <img src="./media/rosbag_odom2.png" height="200">
    </p>
</p>

```bash
# Install packages
sudo apt install ros-eloquent-rosbag2* -y
sudo apt install ros-eloquent-ros2bag -y 

ros2 bag play <your-prefer-rosbag>

# For laser odom viz
ros2 launch neuronbot2_lecture rosbag_odom_rviz.launch.py
# For odom comparison
ros2 launch neuronbot2_sensor_fusion rosbag_odom_rviz.launch.py
```

# Day 3
---



------------------------------------------------------------

```
ros2 launch neuronbot2_lecture spot.launch.py
```

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

# My mannual Conflict
