from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='neuronbot2_sensor_fusion',
            node_executable='gt_odom_node.py',
            parameters=[
                {"model_name": "neuronbot2"},
            ],
            output='screen',
            emulate_tty=True
        )
    ])