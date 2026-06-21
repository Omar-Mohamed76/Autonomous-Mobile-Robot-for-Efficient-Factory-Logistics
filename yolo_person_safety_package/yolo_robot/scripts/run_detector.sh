#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run yolo_robot person_path_detector
