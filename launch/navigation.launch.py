import os
import yaml
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml

def read_worlds_from_yaml():
    package_path = get_package_share_directory('group_project')
    file_path = os.path.join(package_path, 'launch', 'worlds.yaml')
    try:
        with open(file_path, 'r') as file:
            worlds_dict = yaml.safe_load(file)
            return worlds_dict
    except FileNotFoundError:
        print("The specified YAML file was not found.")
    except yaml.YAMLError as exc:
        print("An error occurred while parsing the YAML file:", exc)

def launch_setup(context, *args, **kwargs):
    worlds = read_worlds_from_yaml()
    world_arg = context.launch_configurations['world']

    if world_arg not in worlds.keys():
        sys.exit(f"The world you are trying to access ({world_arg!r}) isn't available. The available options are: {' '.join(list(worlds.keys()))!r}.")

    world_dir = worlds[world_arg]

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # localisation
    navigation_launch_dir = os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'launch')
    map_path = os.path.join(
        get_package_share_directory('group_project'),
        'worlds',
        world_dir,
        'map',
        'map.yaml'
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_launch_dir, 'navigation2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_path
        }.items()
    )

    ld = LaunchDescription([
        navigation_launch
    ])

    return [ld]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            description='Which world to launch.'
        ),

        OpaqueFunction(function=launch_setup)
    ])
