#!/bin/bash
# Build script for catkin_ws
# Uses a cmake stub to prevent Anaconda's protobuf v29.3 from conflicting
# with the system protobuf v3.6.1 required by Gazebo/ignition-transport.

set -e
cd "$(dirname "$0")/.."

catkin_make "$@" -Dprotobuf_DIR="$PWD/cmake"
