# StratomCurtain
Proof-of-concept implementation of a lidar-based object detection and tracking system for Stratom Robotics. Created during Mines CS Field Session.

# Contributors
- Brady Veltien
- Ethan Ko
- Lucas Niewohner
- Ryan Lopez

# Packages

### PyTracker
Gets polygon information from costmap_converter, assigns the polygons ids, and detects any movement with centroid tracking.
This node outputs the location of objects by describing them with encompassing angles and the closest distance to the lidar, the time and frame, and if the object is static or dynamic.

### costmap_converter
A ros package that includes plugins and nodes to convert occupied costmap2d cells to primitive types.
It was modified to work on ros2 humble from https://github.com/rst-tu-dortmund/costmap_converter.

### OdomTf2Adapter
Converts odometry information to a Tf message


# Build
1. Create a Docker image from the Dockerfile
2. Create a Docker container from the image
3. Go to the directory containing the src folder and run colcon build.


# Usage
* ros2 launch stratom_sim stratom_sim.launch.py
    Launches Gazebo simulation and all related nodes
* rviz2
    Used to visualize outputs

### Launch Params:
1. [Optional] Select a different world from stratom_sim/worlds directory<br />
world:=World1

# License:
The py_tracker, statom_sim, and odom_tf2_adapter packages are under the MIT licence
The cost costmap_converter package is under a BSD licence with other 3rd party dependencies that falls under the different licences.
