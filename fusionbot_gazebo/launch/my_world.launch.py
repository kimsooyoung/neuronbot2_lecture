import os
import xacro

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

from osrf_pycommon.terminal_color import ansi

def generate_launch_description():

    # Prepare Robot State Publisher Params
    description_pkg_path = os.path.join(get_package_share_directory('fusionbot_description'))
    gazebo_model_path = os.path.join(description_pkg_path, 'models')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += ":" + gazebo_model_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = gazebo_model_path

    print(ansi("yellow"), "If it's your 1st time to download Gazebo model on your computer, it may take few minutes to finish.", ansi("reset"))

    pkg_path = os.path.join(get_package_share_directory('fusionbot_gazebo'))
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   

    world_path = os.path.join(pkg_path, 'world', 'custom_world')

    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_path}.items()
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

    # Robot State Publisher
    urdf_file = os.path.join(description_pkg_path, 'urdf', 'fusionbot_description.urdf')
    # urdf_file = os.path.join(description_pkg_path, 'urdf', 'skidbot.urdf')
    doc = xacro.parse(open(urdf_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}
    param = {'use_sim_time': True, 'robot_description': doc.toxml()}


    robot_state_publisher = Node(
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        node_name='robot_state_publisher',
        output='screen',
        # parameters=[urdf_file]
        arguments=[urdf_file]
    )

    return LaunchDescription([
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        # joint_state_publisher,
        robot_state_publisher,
    ])
