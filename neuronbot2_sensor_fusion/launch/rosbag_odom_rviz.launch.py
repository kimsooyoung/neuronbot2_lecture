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

    rviz_config_file = os.path.join(get_package_share_directory('neuronbot2_sensor_fusion'), 'rviz', 'neuronbot2_laser_odom_compare.rviz')

    # Launch RViz
    rviz = Node(
        package='rviz2',
        node_executable='rviz2',
        node_name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        rviz,
    ])
