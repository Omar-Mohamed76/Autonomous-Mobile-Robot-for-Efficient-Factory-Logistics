# YOLO Person Safety + MoveRobot Action Client

## Overview

This ROS 2 package provides person detection and robot safety control using a USB webcam, YOLO, and a custom `MoveRobot` action.

It contains two main nodes:

- `person_path_detector_node.py`
  - Reads frames from the webcam.
  - Detects people using YOLO.
  - Checks whether a detected person is inside the robot's path.
  - Publishes `/person_in_path`.
  - Publishes `/vision_command`.

- `person_safety_action_client_node.py`
  - Subscribes to the person safety state.
  - Sends `MoveRobot.Goal(action="start")` to `/start_action`.
  - Cancels the active goal when a person blocks the robot's path.
  - Sends a new start goal after the path becomes clear again.

## System Architecture

```text
USB Webcam
    |
    v
YOLO Person Detection
    |
    v
Path Overlap Evaluation
    |
    +--> /person_in_path   [std_msgs/msg/Bool]
    |
    +--> /vision_command   [std_msgs/msg/String]
             |
             v
Person Safety Action Client
             |
             v
/start_action   [logistic_msg/action/MoveRobot]
```

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- USB webcam available as `/dev/video0`
- YOLO model located at `/home/ubuntu/yolov8n.pt`
- ROS 2 package named `logistic_msg`
- Action interface named `logistic_msg/action/MoveRobot`
- Action server running on `/start_action`

The `MoveRobot` action must contain the following fields:

```text
# Goal
string action
---
# Result
float64 pose_x
float64 pose_y
---
# Feedback
float64 pose_x
float64 pose_y
```

`float32` may also be used instead of `float64`, but the field names must remain:

```text
action
pose_x
pose_y
```

## Package Installation

After extracting the project files, copy the `yolo_robot` package into the ROS 2 workspace:

```bash
mkdir -p ~/ros2_ws/src
cp -r yolo_robot ~/ros2_ws/src/
```

The final package path should be:

```text
~/ros2_ws/src/yolo_robot
```

## Verify the Action Interface

Source ROS 2 and the workspace:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

Check the action interface:

```bash
ros2 interface show logistic_msg/action/MoveRobot
```

Check the available action servers:

```bash
ros2 action list -t
```

The following action should be available:

```text
/start_action [logistic_msg/action/MoveRobot]
```

## Build

```bash
cd ~/ros2_ws

source /opt/ros/humble/setup.bash

rosdep install   --from-paths src   --ignore-src   -r -y

rm -rf build/yolo_robot
rm -rf install/yolo_robot

colcon build   --packages-select yolo_robot   --symlink-install

source ~/ros2_ws/install/setup.bash
```

Verify the installed executables:

```bash
ros2 pkg executables yolo_robot
```

Expected output:

```text
yolo_robot person_path_detector
yolo_robot person_safety_action_client
```

## Run the System

Open three terminals.

### Terminal 1: MoveRobot Action Server

Start the action server used by the robot.

Then verify that it is available:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 action list -t
```

Expected output:

```text
/start_action [logistic_msg/action/MoveRobot]
```

### Terminal 2: Person Path Detector

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run yolo_robot person_path_detector
```

### Terminal 3: Person Safety Action Client

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run yolo_robot person_safety_action_client
```

## Published Topics

### Person Safety State

```text
Topic: /person_in_path
Type: std_msgs/msg/Bool
```

Values:

```text
false = The robot path is clear.
true  = A person is blocking the robot path.
```

Monitor the topic:

```bash
ros2 topic echo /person_in_path
```

### Vision Commands

```text
Topic: /vision_command
Type: std_msgs/msg/String
```

Possible messages:

```text
PERSON_BLOCKING
PATH_CLEAR
```

Monitor the topic:

```bash
ros2 topic echo /vision_command
```

## Action Client Behavior

When the path is clear:

```text
/person_in_path = false
```

The action client sends:

```text
MoveRobot.Goal(action="start")
```

When a person blocks the path:

```text
/person_in_path = true
```

The action client requests cancellation of the active `/start_action` goal.

When the path becomes clear again, the action client sends a new start goal.

## Test the Action Client Without the Camera

Stop the person detector before performing this test.

### Start the Robot

```bash
ros2 topic pub --once   /person_in_path   std_msgs/msg/Bool   "{data: false}"
```

### Simulate a Person Blocking the Path

```bash
ros2 topic pub --once   /person_in_path   std_msgs/msg/Bool   "{data: true}"
```

### Simulate the Path Becoming Clear Again

```bash
ros2 topic pub --once   /person_in_path   std_msgs/msg/Bool   "{data: false}"
```

## Change the Action Server Name

The action server name can be changed using a ROS 2 parameter:

```bash
ros2 run yolo_robot person_safety_action_client   --ros-args   -p action_name:=/start_action
```

## Disable Automatic Start

To prevent the action client from starting the robot automatically when the first clear-path message is received:

```bash
ros2 run yolo_robot person_safety_action_client   --ros-args   -p auto_start_when_clear:=false
```

## Camera Troubleshooting

Do not run more than one process that uses `/dev/video0`.

Check which process is using the webcam:

```bash
sudo fuser -v /dev/video0
```

Stop the process using the webcam:

```bash
sudo fuser -k /dev/video0
```

Check the available camera devices:

```bash
ls -l /dev/video*
```

## Important Safety Note

The `/start_action` action server must support goal cancellation.

When a cancellation request is accepted, the action server must stop the robot motors safely and complete the goal with a canceled status. Accepting the cancellation request without stopping the robot is not sufficient.

The vision system should be used as an additional safety layer and should not be the robot's only emergency-stop mechanism.
