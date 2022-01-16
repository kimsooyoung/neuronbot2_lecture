import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable

from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare

import xacro

def generate_launch_description():
    
    pkg_path = get_package_share_directory('neuronbot2_sensor_fusion')
    ekf_config = os.path.join(pkg_path, 'config', 'ekf.yaml')

    robot_localization = Node(
        package='robot_localization',
        node_executable='ekf_node',
        emulate_tty=True, # Fix Eloquent bug which displays no msgs on console
        output='screen',
        parameters=[ekf_config],
        # remappings=[("odometry/filtered", "odom")]
    )

    rviz_config_file = os.path.join(pkg_path, 'rviz', 'sensor_fusion.rviz')

    # Launch RViz
    rviz = Node(
        package='rviz2',
        node_executable='rviz2',
        node_name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        robot_localization,
        rviz,
    ])