
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Set the path to this package.
    pkg_share = FindPackageShare(package='stratom_sim').find('stratom_sim')

    # Get package share directory for turtlbot3 launch files
    launch_file_dir = os.path.join(pkg_share, 'launch')

    # Hardcode select the TURTLEBOT3_MODEL environment variable to waffle
    os.environ['TURTLEBOT3_MODEL'] = 'waffle'


    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')
    world = LaunchConfiguration('world')

    default_world = "factory.world"
    worlds_path = os.path.join(pkg_share, 'worlds')

    # Declare the launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=default_world,
        description='Full path to the world model file to load')

    # create a launch config variable that includes the load_world_into_gazebo.launch.py file
    load_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'load_world_into_gazebo.launch.py')),
        launch_arguments={'world': PathJoinSubstitution([worlds_path, world])}.items()
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        )
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
    

    # Add the commands to the launch description
    ld = LaunchDescription()
    ld.add_action(declare_world_cmd)

    ld.add_action(load_world)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    
    return ld