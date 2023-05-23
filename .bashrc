source /opt/ros/humble/setup.bash
if [ -d install ]; then
    source install/setup.bash
fi
alias stratbuild="cd ~/mines_ws && source install/setup.bash && colcon build --symlink-install"
alias stratrun="cd ~/mines_ws && source install/setup.bash && ros2 launch stratom_sim turtlebot_sim.launch.py"