#!/usr/bin/env bash
set -euo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"
source "$HOME/PX4_Firmware/Tools/setup_gazebo.bash" \
  "$HOME/PX4_Firmware/" "$HOME/PX4_Firmware/build/px4_sitl_default"
export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:$HOME/PX4_Firmware:$HOME/PX4_Firmware/Tools/sitl_gazebo"
export PYTHONPATH="$WS/scripts:$PYTHONPATH"

trap 'kill 0 2>/dev/null || true' INT TERM EXIT
/usr/bin/python3 "$WS/scripts/ground_station_ros_gateway.py" &
gateway_pid=$!
sleep 1
cd "$WS/ground_station_ui"
npm run dev
kill "$gateway_pid" 2>/dev/null || true
