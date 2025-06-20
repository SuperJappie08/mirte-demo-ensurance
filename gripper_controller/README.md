# Gripper Controller

## Overview

The **Gripper Controller** is a ROS 2 node for controlling the gripper of the Mirte master robot. It provides a simple service interface to open or close the gripper and monitors the gripper's position to confirm successful execution. The node communicates with the gripper hardware using an action client and receives feedback via a position topic.

## Features

- **Service Interface:** Exposes a `toggle_gripper` service (`std_srvs/srv/SetBool`) to open (`true`) or close (`false`) the gripper.
- **Action Client:** Sends position commands to the gripper controller using the `control_msgs/action/GripperCommand` action.
- **Position Monitoring:** Subscribes to the gripper's servo position and checks if the target position is reached within a timeout.
- **Feedback:** Logs progress and reports if the gripper reached the target or timed out.

## Topics and Services

- **Service:** `toggle_gripper` (`std_srvs/srv/SetBool`)
- **Action Client:** `/mirte_master_gripper_controller/gripper_cmd` (`control_msgs/action/GripperCommand`)
- **Subscribes:** `/io/servo/hiwonder/gripper/position` (`mirte_msgs/msg/ServoPosition`)

## Usage

### Building

Make sure your ROS 2 workspace is sourced and build the package:

```bash
colcon build --packages-select gripper_controller
source install/setup.bash
```

### Running

Launch the gripper controller node:

```bash
ros2 run gripper_controller gripper_controller_node
```

### Using the Service

You can open or close the gripper using the ROS 2 service call:

**Open the gripper:**
```bash
ros2 service call /toggle_gripper std_srvs/srv/SetBool "{data: true}"
```

**Close the gripper:**
```bash
ros2 service call /toggle_gripper std_srvs/srv/SetBool "{data: false}"
```

## Code Structure

- `gripper_controller.cpp`: Main implementation of the gripper controller node.

## Main Classes and Methods

- **GripperControllerNode:** Main node class.
  - `handle_gripper_request`: Handles service requests to open/close the gripper.
  - `gripper_position_callback`: Receives and stores the latest gripper position.
  - `check_gripper_progress`: Periodically checks if the gripper reached the target position.

## Requirements

- ROS 2 (tested on Humble)
- `control_msgs` (for GripperCommand action)
- `mirte_msgs` (for ServoPosition message)
- `std_srvs` (for SetBool service)
