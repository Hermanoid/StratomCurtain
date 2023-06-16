import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Set the path to this package.
    pkg_share = FindPackageShare(package="stratom_sim").find("stratom_sim")

    params_file = os.path.join(pkg_share, "config", "default.yaml")

    costmap_converter_cmd = Node(
        package="costmap_converter", executable="standalone_converter", name="costmap_converter", output="screen", parameters=[params_file],
    )

    pytracker_cmd = Node(package="py_tracker", executable="py_tracker", name="py_tracker", parameters=[params_file])

    rviz_cmd = Node(
        package='rviz2', namespace='', executable='rviz2', name='rviz2',
         arguments=['-d',  os.path.join(pkg_share, 'rviz', 'default.rviz') ],
         parameters=[params_file]
        )
    # Add the commands to the launch description
    ld = LaunchDescription()
    ld.add_action(costmap_converter_cmd)
    ld.add_action(pytracker_cmd)
    ld.add_action(rviz_cmd)

    return ld
