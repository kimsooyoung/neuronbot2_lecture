import os

import launch.actions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch.actions import TimerAction, ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node

def generate_launch_description():
    
    rviz_config_dir = os.path.join(get_package_share_directory('neuronbot2_mapping'), 'rviz', 'map.rviz')
    map_file = os.path.join(get_package_share_directory('neuronbot2_mapping'), 'map', 'cartographer_map.yaml')

    map_server = Node(
        package='nav2_map_server',
        node_executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': True}, 
            {'yaml_filename': map_file}
        ]
    )

    rviz2 = Node(
        package='rviz2',
        node_executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        node_executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'autostart': True},
            {'node_names': ['map_server']}
        ]
    )

    map_service_call_param = "{map_url: " + map_file + "}"
    map_service_call = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/load_map', 
            'nav2_msgs/srv/LoadMap', map_service_call_param],
        output='log'
    )

    return LaunchDescription([
        TimerAction(
            period=3.0,
            actions=[map_service_call]
        ),
        # Be Care for order
        rviz2,
        lifecycle_manager,
        map_server,
    ])