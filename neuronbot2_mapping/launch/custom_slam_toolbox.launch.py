import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    open_rviz = LaunchConfiguration('open_rviz', default='false')

    rviz_config_dir = os.path.join(get_package_share_directory('neuronbot2_mapping'), 'rviz', 'slam_toolbox.rviz')
    slam_toolbox_param_dir = os.path.join(get_package_share_directory('neuronbot2_mapping'), 'param', 'slam_toolbox_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'open_rviz',
            default_value='false',
            description='open rviz'),

        launch_ros.actions.Node(
            parameters=[slam_toolbox_param_dir],
            package='slam_toolbox',
            # node_executable='async_slam_toolbox_node',
            node_executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen'
            ),

        Node(
            package='rviz2',
            node_executable='rviz2',
            node_name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration("open_rviz"))
            # output='log'
            ),

        ])
