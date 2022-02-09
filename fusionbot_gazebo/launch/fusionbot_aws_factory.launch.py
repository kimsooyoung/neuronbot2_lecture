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
    robomaker_pkg_path = os.path.join(get_package_share_directory('aws_robomaker_small_warehouse_world'))
    gazebo_model_path = os.path.join(robomaker_pkg_path, 'models')

    fusionbot_pkg_path = os.path.join(get_package_share_directory('fusionbot_description'))
    gazebo_model_path2 = os.path.join(fusionbot_pkg_path, 'models')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += ":" + gazebo_model_path + ":" + gazebo_model_path2
    else:
        os.environ['GAZEBO_MODEL_PATH'] = gazebo_model_path + ":" + gazebo_model_path2

    print(ansi("yellow"), "If it's your 1st time to download Gazebo model on your computer, it may take few minutes to finish.", ansi("reset"))

    pkg_path = os.path.join(get_package_share_directory('fusionbot_gazebo'))
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   

    world_path = os.path.join(pkg_path, 'world', 'fusionbot_factory.world')

    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_path}.items()
    )

    # Start Gazebo client    
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )

    return LaunchDescription([
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
    ])