# Finite State Machine (FSM) Node

## Overview

This document describes the C++ implementation of the Finite State Machine (FSM) for the Apple Picking Task. The FSM manages the robot's high-level behavior, including navigation, apple picking, and returning to the basket.

## States

The FSM cycles through the following states:

- **REST**: Idle state, waiting for a trigger to start the task.
- **NAVIGATING**: Navigates to a tree.
- **SCAN_TREE**: Scans the tree for apples.
- **FINE_NAVIGATION**: Performs fine navigation to approach the apple.
- **OPEN_GRIPPER**: Opens the gripper before picking.
- **ARM_MANIPULATION_TREE**: Moves the arm to the apple's position.
- **GRASP_APPLE**: Closes the gripper to grasp the apple.
- **HOME_ARM_1**: Returns the arm to the home position after picking.
- **GOTO_BASKET**: Navigates to the basket to deposit apples.
- **ARM_MANIPULATION_BASKET**: Moves the arm to the basket position.
- **RELEASE_APPLE**: Opens the gripper to release the apple.
- **HOME_ARM_2**: Returns the arm to the home position after releasing.

## Transitions

- **REST → NAVIGATING**: Triggered by a service request to `/send_state`
- **NAVIGATING → SCAN_TREE**: Upon reaching a tree.
- **SCAN_TREE → FINE_NAVIGATION**: After recieving one apple location through the `/point3d` topic.
- **FINE_NAVIGATION → OPEN_GRIPPER**: After reaching the apple location.
- **OPEN_GRIPPER → ARM_MANIPULATION_TREE**: Gripper is open.
- **ARM_MANIPULATION_TREE → GRASP_APPLE**: Arm in position.
- **GRASP_APPLE → HOME_ARM_1**: Apple grasped.
- **HOME_ARM_1 → GOTO_BASKET**: Arm homed, ready to move.
- **GOTO_BASKET → ARM_MANIPULATION_BASKET**: At basket.
- **ARM_MANIPULATION_BASKET → RELEASE_APPLE**: Arm in position.
- **RELEASE_APPLE → HOME_ARM_2**: Apple released.
- **HOME_ARM_2 → REST**: Task complete or next cycle.

## Key Topics, Actions, and Services

- **Actions**:
  - `navigate_to_pose` (`nav2_msgs::action::NavigateToPose`)
  - `move_arm` ([`fsm::action::MoveArm`](action/MoveArm.action))
- **Services**:
  - `get_point` ([`fsm::srv::GetPoint`](srv/GetPoint.srv))
  - `toggle_gripper` (`std_srvs::srv::SetBool`)
  - `send_state` ([`fsm::srv::SendState`](srv/SendState.srv))
- **Topics**:
  - `/point3d` (`geometry_msgs/msg/Point`): Receives detected apple positions.
  - `/fsm_debug` (`std_msgs/msg/String`): Publishes debug messages.

## Parameters

- `tree_pose1`, `tree_pose2`, `tree_pose3`: Tree positions (as arrays of doubles).
- `basket`: Basket position (as array of doubles).

These are defined in the `fsm_params.yaml' in the form of (x,y) and quaternions (z,w) assuming only yaw for a planar robot. These also also defined in the map frame. 

## Usage

**Build the package** (from the workspace root):

 ```sh
 colcon build --packages-select fsm
```
**Run the node**
```sh
ros2 run fsm fsm_node --ros-args --params-file $(ros2 pkg prefix fsm)/share/fsm/config/fsm_params.yaml
```
**Trigger start** or any other state manually
```sh
ros2 service call /send_state fsm/srv/SendState "{command: 'NAVIGATING'}"
```

Implementation Notes:
The FSM logic is implemented in fsm.cpp.
The node uses ROS2 actions for navigation and arm control, and services for gripper and state management.
Debug output is published to /fsm_debug for monitoring.
