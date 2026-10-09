#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash
source "$HOME/catkin_ws/devel/setup.bash"
source "$HOME/PX4_Firmware/Tools/setup_gazebo.bash" \
  "$HOME/PX4_Firmware/" "$HOME/PX4_Firmware/build/px4_sitl_default"
export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:$HOME/PX4_Firmware:$HOME/PX4_Firmware/Tools/sitl_gazebo"

exec roslaunch px4 raicom.launch "$@"
