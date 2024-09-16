import os
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

    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')

    world = os.path.join(
        get_package_share_directory('group_project'),
        'worlds',
        world_dir,
        'gazebo.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )


    ld = LaunchDescription()
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)

    return [ld]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            description='Which world to launch.'
        ),

        OpaqueFunction(function=launch_setup)
    ])
