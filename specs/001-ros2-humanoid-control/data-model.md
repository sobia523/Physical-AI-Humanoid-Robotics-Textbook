# Data Model: ROS 2 Humanoid Control Module (Conceptual Entities)

**Branch**: `001-ros2-humanoid-control` | **Date**: 2025-12-11 | **Spec**: `specs/001-ros2-humanoid-control/spec.md`
**Input**: Feature specification and implementation plan for the ROS 2 Humanoid Control Module.

## Overview

For an educational module focused on concepts and practical application of existing technologies, the "data model" refers to the core conceptual entities that are taught and interacted with, rather than a data schema for a software system. These entities define the domain knowledge covered in the module.

## Conceptual Entities

The following are the key conceptual entities central to understanding and implementing ROS 2-based humanoid robot control, as derived from the feature specification:

### ROS 2 Node
*   **Description**: An executable process that performs computation, forming the fundamental unit of computation in ROS 2.
*   **Key Attributes (Conceptual)**:
    *   `name`: Unique identifier for the node within the ROS 2 graph.
    *   `purpose`: The specific task or function it performs (e.g., sensor driver, motor controller, AI decision-maker).
    *   `communication_interfaces`: Topics it publishes/subscribes to, services it provides/uses, actions it acts upon.
*   **Relationships**: Interacts with other Nodes via Topics, Services, and Actions.

### ROS 2 Topic
*   **Description**: A named bus over which nodes exchange messages asynchronously, implementing a publish/subscribe communication pattern.
*   **Key Attributes (Conceptual)**:
    *   `name`: Unique identifier for the topic.
    *   `message_type`: The type of data structure being exchanged.
    *   `publishers`: Nodes that send data to this topic.
    *   `subscribers`: Nodes that receive data from this topic.
*   **Relationships**: Connects publishing Nodes to subscribing Nodes.

### ROS 2 Service
*   **Description**: A named request/reply mechanism for synchronous communication between nodes, used for short-duration, blocking operations.
*   **Key Attributes (Conceptual)**:
    *   `name`: Unique identifier for the service.
    *   `request_type`: The data structure for the service request.
    *   `response_type`: The data structure for the service response.
    *   `server_node`: The Node that provides the service.
    *   `client_nodes`: Nodes that call the service.
*   **Relationships**: Connects a client Node to a server Node for a one-off interaction.

### ROS 2 Action
*   **Description**: A long-running goal-feedback-result communication pattern, suitable for complex, pre-emptible tasks (e.g., navigating a robot to a goal).
*   **Key Attributes (Conceptual)**:
    *   `name`: Unique identifier for the action.
    *   `goal_type`: The data structure for the task goal.
    *   `feedback_type`: The data structure for ongoing progress updates.
    *   `result_type`: The data structure for the final outcome.
    *   `action_server_node`: The Node that executes the action.
    *   `action_client_nodes`: Nodes that request the action.
*   **Relationships**: Coordinates complex tasks between client and server Nodes.

### ROS 2 Message
*   **Description**: A structured data type used for communication over topics, services (request/response), and actions (goal/feedback/result).
*   **Key Attributes (Conceptual)**:
    *   `fields`: Typed variables that compose the message structure (e.g., `int32 x`, `string data`).
    *   `data_purpose`: What the message conveys (e.g., sensor reading, joint command).
*   **Relationships**: Carried by Topics, Services, and Actions.

### Python Agent
*   **Description**: A software component, typically written in Python, that can embody AI logic or decision-making and interacts with the ROS 2 system.
*   **Key Attributes (Conceptual)**:
    *   `logic`: The algorithms or rules that drive its behavior.
    *   `interface_points`: How it connects to ROS 2 (e.g., `rclpy` publishers/subscribers).
*   **Relationships**: Controls or monitors ROS 2 Nodes.

### rclpy
*   **Description**: The official Python client library for ROS 2, enabling Python programs to create ROS 2 Nodes and interact with the ROS 2 graph.
*   **Key Attributes (Conceptual)**:
    *   `API_surface`: Functions and classes for ROS 2 interaction (e.g., `Node`, `Publisher`, `Subscriber`, `Service`, `ActionClient`).
*   **Relationships**: Provides the programming bridge between Python Agents and the ROS 2 system.

### URDF (Unified Robot Description Format)
*   **Description**: An XML-based file format used to describe all aspects of a robot, including its kinematic and dynamic properties, visual appearance, and collision behavior.
*   **Key Attributes (Conceptual)**:
    *   `links`: Rigid bodies of the robot.
    *   `joints`: Connections between links, defining degrees of freedom.
    *   `origin`: Position and orientation of components.
    *   `visuals`: Graphical representation.
    *   `collisions`: Collision geometry.
    *   `inertials`: Mass and inertia properties.
*   **Relationships**: Defines the physical and kinematic structure of a Humanoid Robot Model.

### Humanoid Robot Model
*   **Description**: A digital representation of a human-like robot, typically defined using URDF, used for simulation and control development.
*   **Key Attributes (Conceptual)**:
    *   `kinematics`: Joint relationships and movement.
    *   `dynamics`: Physical properties like mass, inertia.
    *   `sensors`: Simulated sensors (e.g., cameras, IMUs).
*   **Relationships**: Implemented using URDF; interacts with Simulation Environment and ROS 2 Nodes.

### Simulation Environment
*   **Description**: A software platform (e.g., Gazebo, NVIDIA Isaac Sim, Unity) that provides a virtual world for testing robot models and control algorithms.
*   **Key Attributes (Conceptual)**:
    *   `physics_engine`: For realistic physical interactions.
    *   `rendering_engine`: For visualization.
    *   `ROS_interface`: For integration with ROS 2.
*   **Relationships**: Hosts Humanoid Robot Models and provides a testing ground for ROS 2 control systems.
