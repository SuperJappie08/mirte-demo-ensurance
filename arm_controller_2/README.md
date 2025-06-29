# Arm Controller

## Overview

The **Arm Controller** is a ROS 2 node that implements an action server for controlling the Mirte master's robotic arm. It receives target positions (either in Cartesian coordinates or a "home" command), transforms them into the robot's base frame, computes the necessary joint angles using inverse kinematics, and sends joint trajectory commands to the arm controller. The node provides feedback and result messages to clients and monitors the arm's progress toward the goal.

## Features

- **MoveArm Action Server:** Accepts goals to move the arm to a specified (x, y, z) position or to a predefined "home" position.
- **TF2 Integration:** Transforms target coordinates from the `map` frame to the `base_link` frame.
- **Inverse Kinematics:** Uses an IK solver to compute joint angles for the desired end-effector position.
- **Trajectory Control:** Publishes joint trajectories to the arm controller.
- **Feedback and Monitoring:** Provides real-time feedback and monitors the arm's progress, reporting success or failure.
- **Thread Safety:** Uses mutexes to safely handle joint state updates.

## Topics and Actions

- **Action Server:** `/move_arm` (`fsm/action/MoveArm`)
- **Publishes:** `/mirte_master_arm_controller/joint_trajectory` (`trajectory_msgs/msg/JointTrajectory`)
- **Subscribes:** `/joint_states` (`sensor_msgs/msg/JointState`)

## Usage

### Building

Make sure your ROS 2 workspace is sourced and build the package:

```bash
colcon build --packages-select arm_controller
source install/setup.bash
```

### Running

Launch the arm controller node:

```bash
ros2 run arm_controller arm_controller_node
```

### Sending a Goal

You can send a goal to the action server using the ROS 2 CLI or a custom client. Example (replace with your action definition):

```bash
ros2 action send_goal /move_arm fsm/action/MoveArm "{x: 0.2, y: 0.1, z: 0.3, home: false}"
```

To send the arm to the home position set the home field to true:

```bash
ros2 action send_goal /move_arm fsm/action/MoveArm "{x: 0.2, y: 0.1, z: 0.3, home: true}"
```

## Code Structure

- `arm_controller.cpp`: Main implementation of the MoveArm action server.
- `ik_solver.hpp/cpp`: Inverse kinematics solver for the arm (must be present in the same package).

## Main Classes and Methods

- **MoveArmServer:** Main node class implementing the action server.
  - `handle_goal`: Accepts or rejects incoming goals.
  - `handle_cancel`: Handles cancel requests.
  - `handle_accepted`: Starts execution of accepted goals.
  - `execute`: Transforms coordinates, computes IK, sends trajectories, and monitors progress.
  - `send_joint_trajectory`: Publishes joint trajectory messages.
  - `joint_state_callback`: Updates current joint positions.
  - `get_current_joint_positions`: Thread-safe access to joint positions.

## Inverse Kinematics Solver

This package includes an inverse kinematics (IK) solver for the Mirte master robotic arm, implemented in `ik_solver.hpp/cpp`. The IK solver is built using the [OROCOS KDL library](https://www.orocos.org/kdl), following the approach described in Arne Baeyens' tutorial on KDL-based IK solvers. The solver loads the robot's URDF, constructs the kinematic chain, and computes joint angles for a given end-effector position.

**Reference:**  
Arne Baeyens, "KDL Inverse Kinematics Tutorial", available at:  
https://arnebaeyens.com/blog/2024/kdl-ros2/

**Dependencies:**
- [orocos_kdl](https://github.com/orocos/orocos_kinematics_dynamics)
- [kdl_parser](http://wiki.ros.org/kdl_parser)
- [ament_index_cpp](https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html#ament_index_cpp)

The solver is initialized at runtime and uses the robot's URDF to ensure accurate kinematics for the Mirte Master. The solver tries to find the "gripper_center", which is not in the Mirte's URDF by default but needs to be added locally.

## Requirements

- ROS 2 (tested on Humble)
- TF2
- `fsm/action/MoveArm` action definition
- `ik_solver.hpp/cpp` for inverse kinematics
