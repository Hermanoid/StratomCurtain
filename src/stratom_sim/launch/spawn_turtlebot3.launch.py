# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Note: This file was grabbed from the turtlebot3_gazebo package and slightly changed for Stratom's purposes

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the urdf file
    TURTLEBOT3_MODEL = os.environ["TURTLEBOT3_MODEL"]
    model_folder = "turtlebot3_" + TURTLEBOT3_MODEL
    urdf_path = os.path.join(get_package_share_directory("stratom_sim"), "models", model_folder, "model.sdf")

    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration("x_pose", default="0.0")
    y_pose = LaunchConfiguration("y_pose", default="0.0")

    # Declare the launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    declare_x_position_cmd = DeclareLaunchArgument("x_pose", default_value="0.0", description="Specify namespace of the robot")

    declare_y_position_cmd = DeclareLaunchArgument("y_pose", default_value="0.0", description="Specify namespace of the robot")

    start_gazebo_ros_spawner_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", TURTLEBOT3_MODEL, "-file", urdf_path, "-x", x_pose, "-y", y_pose, "-z", "0.01"],
        output="screen",
    )

    # Add odom_tf2_adapter node to convert ground-truth simulation pose information to tf transform
    # This node basically says "Take the position from the simulation and use it directly"
    odom_tf2_adapter_node = Node(
        package="odom_tf2_adapter",
        executable="odom_tf2_adapter",
        name="odom_tf2_adapter",
        parameters=[{"odom_topic": "/odom", "base_frame": "base_footprint", "parent_frame": "odom"}],
    )

    static_tf2_broadcaster_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        arguments=["--frame-id", "map", "--child-frame-id", "odom"],
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)

    # Add any conditioned actions
    ld.add_action(start_gazebo_ros_spawner_cmd)

    # Add the nodes to launch
    ld.add_action(odom_tf2_adapter_node)
    ld.add_action(static_tf2_broadcaster_node)

    return ld
