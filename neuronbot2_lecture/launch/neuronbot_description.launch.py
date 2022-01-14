import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, TimerAction

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro

def generate_launch_description():

    pkg_path = os.path.join(get_package_share_directory('neuronbot2_lecture'))
    description_pkg_path = os.path.join(get_package_share_directory('neuronbot2_description'))

    # Prepare Robot State Publisher Params
    urdf_file = os.path.join(description_pkg_path, 'urdf', 'neuronbot2.urdf')

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        node_executable='joint_state_publisher_gui',
        node_name='joint_state_publisher_gui'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        node_name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[urdf_file],
    )

    rviz_config_file = os.path.join(pkg_path, 'rviz', 'description.rviz')

    # Launch RViz
    rviz = Node(
        package='rviz2',
        node_executable='rviz2',
        node_name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        joint_state_publisher_gui,
        robot_state_publisher,
        TimerAction(    
            period=3.0,
            actions=[rviz]
        ),
    ])