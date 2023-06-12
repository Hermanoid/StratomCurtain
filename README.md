# StratomCurtain
Proof-of-concept implementation of a lidar-based object detection and tracking system for Stratom Robotics. Created during Mines CS Field Session.

# Contributors
- Brady Veltien
- Ethan Ko
- Lucas Niewohner
- Ryan Lopez

# Packages

### PyTracker
A node that tracks polygons using the Hungarian algorithm and detects movement in the polygons with centroid tracking.

### costmap_converter
A ros package that includes plugins and nodes to convert occupied costmap2d cells to primitive types.
It was modified to work on ros2 humble from https://github.com/rst-tu-dortmund/costmap_converter.

### OdomTf2Adapter
Converts odometry information to a Tf message.


# Build
This was build for a docker container running on a Windows computer.
1. Install docker: https://docs.docker.com/desktop/install/windows-install/
2. Update WSL: wsl –update
3. Open powershell and go to the directory you want to download the repository
4. Run the command \<Command\>
5. Go into the directory
6. Buiold the image with the command: docker build -t \<Image_name\> .
7. Create a container from the image with the command: docker run -it --net host --ipc host --gpus=all --privileged -v C:\<path>\<to>\<folder>:/home/mines/mines_ws --name \<Container_name\> \<Image_name\>
8. Run colcon build


# Usage
* ros2 launch stratom_sim stratom_sim.launch.py<br>
    Launches Gazebo simulation and all related nodes
    1. [Optional] Select a different world from the stratom_sim/worlds directory
    world:=World1
* ros2 launch stratom_sim stratom_curtain.launch.py<br>
    Launches all nodes needed to read from a lidar scan and odemetry and output warning messages

# License:
The py_tracker, statom_sim, and odom_tf2_adapter packages are under the MIT licence
The cost costmap_converter package is under a BSD licence with other 3rd party dependencies that falls under the different licences.
