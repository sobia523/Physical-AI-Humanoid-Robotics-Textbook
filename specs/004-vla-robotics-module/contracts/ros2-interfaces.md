# ROS 2 Interfaces for the VLA Robotics Module

**Date**: 2025-12-15
**Feature**: `004-vla-robotics-module`

This document defines the ROS 2 interfaces (topics, services, and actions) that will be used for communication between the different components of the Vision-Language-Action (VLA) pipeline.

---

## 1. Voice Command Processing

### Topics

-   **/voice_audio** (`audio_common_msgs/AudioData`)
    -   **Description**: Publishes raw audio data captured from the microphone.
    -   **Publisher**: `voice_capture_node`
    -   **Subscriber**: `voice_transcription_node`

-   **/transcribed_text** (`std_msgs/String`)
    -   **Description**: Publishes the transcribed text from the voice command.
    -   **Publisher**: `voice_transcription_node`
    -   **Subscriber**: `cognitive_planner_node`

---

## 2. Cognitive Planning

### Services

-   **/generate_task_plan** (`custom_interfaces/GenerateTaskPlan`)
    -   **Description**: A service to request a task plan from the cognitive planner node.
    -   **Request**: `string command` - The transcribed text command.
    -   **Response**: `custom_interfaces/TaskPlan plan` - The generated task plan.
    -   **Server**: `cognitive_planner_node`
    -   **Client**: A user interface or a master control node.

### Topics

-   **/task_plan** (`custom_interfaces/TaskPlan`)
    -   **Description**: Publishes the generated task plan for the execution system.
    -   **Publisher**: `cognitive_planner_node`
    -   **Subscriber**: `task_executor_node`

---

## 3. Perception

### Topics

-   **/detected_objects** (`custom_interfaces/ObjectDetections`)
    -   **Description**: Publishes a list of objects detected in the environment, including their ID, type, position, and orientation.
    -   **Publisher**: `perception_node`
    -   **Subscribers**: `cognitive_planner_node`, `task_executor_node`

---

## 4. Task Execution

### Actions

-   **/navigate_to** (`nav2_msgs/NavigateToPose`)
    -   **Description**: An action to command the robot to navigate to a specific pose in the world. This will use the standard Nav2 action interface.
    -   **Action Server**: `Nav2 Stack`
    -   **Action Client**: `task_executor_node`

-   **/manipulate_object** (`custom_interfaces/ManipulateObject`)
    -   **Description**: A custom action to perform a manipulation task, such as grasping or releasing an object.
    -   **Goal**:
        -   `string action_type` - e.g., "grasp", "release"
        -   `string object_id` - The ID of the object to interact with.
        -   `geometry_msgs/Pose target_pose` - (Optional) A target pose for the end-effector.
    -   **Result**:
        -   `bool success` - Indicates if the manipulation was successful.
    -   **Feedback**:
        -   `string status` - The current stage of the manipulation (e.g., "moving_arm", "closing_gripper").
    -   **Action Server**: `manipulation_controller_node`
    -   **Action Client**: `task_executor_node`

---

## Custom Message Definitions (`custom_interfaces`)

-   **`TaskPlan.msg`**:
    ```
    string id
    string source_command
    Action[] actions
    string status
    ```

-   **`Action.msg`**:
    ```
    string type
    string[] parameters_keys
    string[] parameters_values
    string status
    ```

-   **`ObjectDetections.msg`**:
    ```
    std_msgs/Header header
    SimulatedObject[] objects
    ```

-   **`SimulatedObject.msg`**:
    ```
    string id
    string type
    geometry_msgs/Pose pose
    # ... other properties like color, size, etc.
    ```
-   **`GenerateTaskPlan.srv`**:
    ```
    string command
    ---
    custom_interfaces/TaskPlan plan
    ```
-   **`ManipulateObject.action`**:
    ```
    string action_type
    string object_id
    geometry_msgs/Pose target_pose
    ---
    bool success
    ---
    string status
    ```
