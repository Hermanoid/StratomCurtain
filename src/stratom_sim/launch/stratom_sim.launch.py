import os
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Set the path to this package.
    pkg_share = FindPackageShare(package="stratom_sim").find("stratom_sim")

    # Get package share directory for turtlbot3 launch files
    launch_file_dir = os.path.join(pkg_share, "launch")

    # Hardcode select the TURTLEBOT3_MODEL environment variable to waffle
    os.environ["TURTLEBOT3_MODEL"] = "waffle"

    x_pose = LaunchConfiguration("x_pose", default="-2.0")
    y_pose = LaunchConfiguration("y_pose", default="-0.5")
    world = LaunchConfiguration("world")

    default_world = "World1"
    worlds_path = os.path.join(pkg_share, "worlds")

    params_file = os.path.join(pkg_share, "config", "default.yaml")

    lc = LaunchContext()
    humble_share = "/opt/ros/humble/share"
    gazebo_env_cmd = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=[
            EnvironmentVariable("GAZEBO_MODEL_PATH", default_value="").perform(lc),
            os.pathsep,
            os.path.join(pkg_share, "models/turtlebot3_waffle"),
            os.pathsep,
            humble_share,
        ],
    )

    # Declare the launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        name="world", default_value=default_world, description="Full path to the world model file to load"
    )

    # # create a launch config variable that includes the load_world_into_gazebo.launch.py file
    load_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "load_world_into_gazebo.launch.py")),
        launch_arguments={"world": PathJoinSubstitution([worlds_path, world])}.items(),
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_file_dir, "robot_state_publisher.launch.py"))
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_file_dir, "spawn_turtlebot3.launch.py")),
        launch_arguments={"x_pose": x_pose, "y_pose": y_pose}.items(),
    )

    costmap_converter_cmd = Node(
        package="costmap_converter", executable="standalone_converter", name="costmap_converter", output="screen", parameters=[params_file]
    )

    pytracker_cmd = Node(package="py_tracker", executable="py_tracker", name="py_tracker_node")

    # Add the commands to the launch description
    ld = LaunchDescription()
    ld.add_action(gazebo_env_cmd)
    ld.add_action(declare_world_cmd)

    ld.add_action(load_world)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)

    ld.add_action(costmap_converter_cmd)
    ld.add_action(pytracker_cmd)

    return ld
