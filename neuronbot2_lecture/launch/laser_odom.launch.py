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

    rf2o_laser_odometry_node =Node(
        package='rf2o_laser_odometry',
        node_executable='rf2o_laser_odometry_node',
        node_name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic' : '/scan',
            'odom_topic' : '/odom_rf2o',
            'publish_tf' : False,
            'base_frame_id' : 'base_link',
            'odom_frame_id' : 'odom',
            'init_pose_from_topic' : '',
            'freq' : 10.0
        }],
    )

    rviz_config_file = os.path.join(get_package_share_directory('neuronbot2_lecture'), 'rviz', 'laser_odom.rviz')

    # Launch RViz
    rviz = Node(
        package='rviz2',
        node_executable='rviz2',
        node_name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        rf2o_laser_odometry_node,
        rviz,
    ])
