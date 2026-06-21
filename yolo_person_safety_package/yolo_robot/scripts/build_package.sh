#!/usr/bin/env bash
set -e

cd ~/ros2_ws
source /opt/ros/humble/setup.bash

rosdep install --from-paths src --ignore-src -r -y

rm -rf build/yolo_robot install/yolo_robot

colcon build   --packages-select yolo_robot   --symlink-install

source ~/ros2_ws/install/setup.bash

echo
echo "Build completed."
ros2 pkg executables yolo_robot
