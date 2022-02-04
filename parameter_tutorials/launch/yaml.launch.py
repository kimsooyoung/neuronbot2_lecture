import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    config = os.path.join(
        get_package_share_directory('parameter_tutorials'),
        'config',
        'example.yaml'
    )

    node = Node(
        package='parameter_tutorials',
        node_executable='basic_param_node.py',
        parameters=[config],
        output='screen',
        emulate_tty=True
    )

    ld = LaunchDescription()
    ld.add_action(node)

    return ld