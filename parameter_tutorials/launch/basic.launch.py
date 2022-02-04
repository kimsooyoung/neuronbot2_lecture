from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='parameter_tutorials',
            node_executable='basic_param_node.py',
            parameters=[
                {'my_str': 'Hello world'},
                {'my_int': 5},
                {'my_double_array': [4.4, 5.5, 6.6]}
            ],
            output='screen',
            emulate_tty=True
        )
    ])