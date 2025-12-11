# Module 1: The Robotic Nervous System (ROS 2)

## Overview

This module is designed to provide advanced AI & robotics students, developers, and practitioners with a foundational understanding and practical skills for building and simulating humanoid robot control pipelines using ROS 2. It covers the core concepts of ROS 2 middleware, bridging Python agents to ROS, and creating URDF-based humanoid models.

## Module Content Highlights

*   **Chapter 1: Introduction to ROS 2**: Explores the ROS 2 architecture, fundamental concepts (nodes, topics, services, actions), and communication patterns.
*   **Chapter 2: ROS 2 Nodes and Topics**: Details how to create ROS 2 nodes in Python and implement basic publish-subscribe communication for data exchange.
*   **Chapter 3: ROS 2 Services and Actions**: Focuses on implementing client-server communication using ROS 2 services and actions for command-response behaviors and long-running tasks.
*   **Chapter 4: Bridging Python Agents to ROS 2**: Demonstrates how to integrate Python-based AI agents with ROS 2 using `rclpy` to control robot joint movements.
*   **Chapter 5: URDF for Humanoids and Simulation**: Covers understanding and creating URDF models for humanoid robots and their integration with ROS 2 simulation environments like Rviz.

## Setup Instructions

To get the code examples and exercises in this module running on your system, please follow these steps.

### Prerequisites

Ensure you have the following installed:

1.  **Ubuntu 22.04 LTS (Jammy Jellyfish)**: Recommended operating system for ROS 2 development.
2.  **ROS 2 Humble Hawksbill (or Iron Irwini)**: Follow the official ROS 2 documentation for installation instructions:
    *   [ROS 2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation.html)
    *   [ROS 2 Iron Installation Guide](https://docs.ros.org/en/iron/Installation.html)
    *   Ensure you install the `ros-humble-desktop` (or `ros-iron-desktop`) package, which includes `rclpy`, `rviz`, `gazebo`, and other essential tools.
3.  **Python 3.10+**: Typically comes pre-installed with Ubuntu 22.04 and is required for `rclpy`.
4.  **Colcon**: The ROS 2 build tool, usually installed with the desktop version of ROS 2.
5.  **Git**: For cloning the repository.

### Getting Started

1.  **Clone the Repository**:
    ```bash
    git clone https://github.com/your-organization/Physical-AI-Humanoid-Robotics-Textbook.git
    cd Physical-AI-Humanoid-Robotics-Textbook
    ```
2.  **Navigate to the Module Directory**:
    ```bash
    cd module1-ros2-humanoid-control
    ```
3.  **Build the ROS 2 Packages**:
    This module is structured as a ROS 2 workspace. Source your ROS 2 installation and then build the packages:
    ```bash
    # Source your ROS 2 installation (replace 'humble' with your ROS 2 distro if different)
    source /opt/ros/humble/setup.bash

    # Build the workspace
    colcon build --symlink-install
    ```
4.  **Source the Workspace**:
    After building, you need to source the workspace to make the new ROS 2 packages available to your environment:
    ```bash
    source install/setup.bash
    ```

## Running Examples

Each chapter's content provides specific instructions for running its associated code examples. Here are general commands you might use, with detailed usage in the respective chapters:

*   **Chapter 2 (Simple Publisher/Subscriber):**
    ```bash
    ros2 launch module1_ros2_humanoid_control simple_comm_launch.py
    ```
*   **Chapter 3 (Gripper Service/Arm Action):**
    *   Service Example:
        ```bash
        # Terminal 1:
        ros2 run module1_ros2_humanoid_control gripper_service_server
        # Terminal 2:
        ros2 run module1_ros2_humanoid_control gripper_service_client true # or false
        ```
    *   Action Example:
        ```bash
        # Terminal 1:
        ros2 run module1_ros2_humanoid_control arm_action_server
        # Terminal 2:
        ros2 run module1_ros2_humanoid_control arm_action_client 2.0 # (target position)
        ```
*   **Chapter 4 (Python Agent Control):**
    ```bash
    ros2 launch module1_ros2_humanoid_control agent_control_launch.py
    ```
*   **Chapter 5 (URDF Visualization in Rviz):**
    ```bash
    ros2 launch module1_ros2_humanoid_control display_arm_launch.py
    ```

Refer to the individual chapter markdown files (`content/chapter*.md`) for complete explanations and execution details for each example.
