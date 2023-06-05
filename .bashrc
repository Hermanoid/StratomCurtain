export TURTLEBOT3_MODEL=waffle
source /opt/ros/humble/setup.bash
if [ -d install ]; then
    source install/setup.bash
fi
alias stratobuild="cd ~/mines_ws && source install/setup.bash && colcon build --symlink-install"
alias stratolaunch="cd ~/mines_ws && source install/setup.bash && ros2 launch stratom_sim stratom_sim.launch.py"
alias stratocontrol="cd ~/mines_ws && source install/setup.bash && ros2 run turtlebot3_teleop teleop_keyboard"
