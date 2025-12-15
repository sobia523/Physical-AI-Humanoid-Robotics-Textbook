# Quickstart: Vision-Language-Action (VLA) Robotics Module

**Date**: 2025-12-15
**Feature**: `004-vla-robotics-module`

This document provides a brief overview of how to get the VLA module running.

## Prerequisites

1.  **ROS 2 Humble/Iron**: Ensure you have a working installation of ROS 2.
2.  **Gazebo/Unity**: Have Gazebo or Unity installed and configured to work with ROS 2.
3.  **OpenAI API Key**: Obtain an API key from OpenAI to use their language models.
4.  **Python 3.10+**: With the necessary libraries (`openai`, `rclpy`, etc.).
5.  **Microphone**: A working microphone for voice commands.

## Setup

1.  **Source the ROS 2 Environment**:
    ```bash
    source /opt/ros/humble/setup.bash
    ```

2.  **Set OpenAI API Key**:
    ```bash
    export OPENAI_API_KEY='your-api-key'
    ```

3.  **Build and Source the Workspace**:
    Navigate to your ROS 2 workspace containing this module and run:
    ```bash
    colcon build --packages-select vla_robotics_package
    source install/setup.bash
    ```

## Running the VLA Pipeline

The VLA pipeline is launched using a single ROS 2 launch file.

1.  **Launch the Simulation and VLA Nodes**:
    ```bash
    ros2 launch vla_robotics_package vla_pipeline.launch.py
    ```

    This launch file will start:
    -   The Gazebo/Unity simulation environment with the humanoid robot.
    -   The `voice_capture_node` to listen for microphone input.
    -   The `voice_transcription_node` to convert speech to text.
    -   The `cognitive_planner_node` to generate a task plan from the text.
    -   The `task_executor_node` to execute the plan.
    -   The `perception_node` to detect objects in the simulation.

2.  **Give a Voice Command**:
    Once the simulation is running, simply speak a command into your microphone, for example:
    > "Pick up the red cube and move it to the blue table."

3.  **Observe the Robot**:
    Watch the simulated humanoid robot as it:
    -   Receives the command.
    -   (Optionally) Displays the transcribed text and the generated plan in the terminal.
    -   Navigates to the red cube.
    -   Picks it up.
    -   Navigates to the blue table.
    -   Places the cube on the table.

## Key Components

-   **Voice Capture**: Listens for audio and publishes it.
-   **Transcription**: Uses OpenAI Whisper to convert audio to text.
-   **Cognitive Planner**: Sends the text to a GPT model to get a sequence of actions.
-   **Task Executor**: Translates the action sequence into ROS 2 goals (navigation, manipulation).
-   **Perception**: Identifies objects in the simulation.
-   **Simulation**: The Gazebo or Unity world where the robot exists.
