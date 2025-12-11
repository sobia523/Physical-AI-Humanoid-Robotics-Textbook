# Contracts: Module 2 — The Digital Twin (Gazebo & Unity)

**Branch**: `001-digital-twin-module` | **Date**: 2025-12-11
**Context**: Definition of communication interfaces and data structures used within the 'Digital Twin Module'.

## Overview

For this module, the "contracts" are primarily defined by the standard ROS 2 message types, service definitions, and action definitions. These definitions dictate the structure of data exchanged between ROS 2 nodes, including those interacting with Gazebo and Unity.

As these are external definitions managed by the ROS 2 ecosystem, explicit contract files (like OpenAPI/GraphQL schemas) are not generated here. Instead, adherence to the specified ROS 2 message types is the contract.

## Conceptual Example: `SetGripperState.srv`

This is a conceptual example of a ROS 2 service contract that might be used to set the state of a robotic gripper.

```ros2
# Request
bool gripper_open # true to open the gripper, false to close
---
# Response
bool success      # true if the gripper state was set successfully, false otherwise
string message    # A message providing more details about the operation
```

## Conceptual Example: `MoveArm.action`

This is a conceptual example of a ROS 2 action contract that might be used to move a robotic arm to a target position.

```ros2
# Goal
geometry_msgs/msg/Point target_position
geometry_msgs/msg/Quaternion target_orientation
---
# Result
bool success
string message
---
# Feedback
float32 distance_to_target
```