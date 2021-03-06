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
    world = [get_package_share_directory('neuronbot2_gazebo'), '/worlds/']
    world.append(LaunchConfiguration('world_model', default='mememan_world.model'))

    gazebo_model_path = os.path.join(get_package_share_directory('neuronbot2_gazebo'), 'models')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += ":" + gazebo_model_path
    else :
        os.environ['GAZEBO_MODEL_PATH'] = gazebo_model_path

    print(os.environ['GAZEBO_MODEL_PATH'])

    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   

    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        # launch_arguments={'world': world_path}.items()
    )

    # Start Gazebo client    
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        node_executable='joint_state_publisher',
        node_name='joint_state_publisher'
    )

    # Prepare Robot State Publisher Params
    description_pkg_path = os.path.join(get_package_share_directory('neuronbot2_description'))
    urdf_file = os.path.join(description_pkg_path, 'urdf', 'neuronbot2.urdf')

    # Robot State Publisher
    doc = xacro.parse(open(urdf_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}
    param = {'use_sim_time': True, 'robot_description': doc.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        node_name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[urdf_file],
    )

    # Spawn Robot
    spawn_entity = Node(
        package='gazebo_ros', 
        node_executable='spawn_entity.py',
        output='screen',
        arguments=['-topic', 'robot_description', '-entity', 'neuronbot2'],
    )

    return LaunchDescription([
        # ExecuteProcess(
        #     cmd=['gzserver', '--verbose' , '-s', 'libgazebo_ros_init.so'],
        #     output='screen'
        # ),
        # ExecuteProcess(
        #     cmd=['gzclient'],
        #     output='screen'
        # ),
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        joint_state_publisher,
        robot_state_publisher,
        spawn_entity,
    ])
