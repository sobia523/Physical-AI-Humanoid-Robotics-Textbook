# Quickstart: Digital Twin Module

**Branch**: `002-digital-twin-module` | **Date**: 2025-12-12 | **Spec**: specs/002-digital-twin-module/spec.md
**Input**: Feature specification from `specs/002-digital-twin-module/spec.md`

## Overview

This Quickstart guide provides a rapid introduction to Module 2, "The Digital Twin (Gazebo & Unity)," for students with foundational knowledge of ROS 2 and humanoid robot middleware. It outlines the essential steps to get started with simulating humanoid robots in virtual environments using Gazebo for physics simulation and Unity for high-fidelity rendering.

## Prerequisites

Before proceeding, ensure you have the following installed and configured on your system:

1.  **ROS 2**: Humble or Iron distribution (Linux recommended).
    *   Familiarity with ROS 2 concepts (nodes, topics, services, actions) is assumed.
2.  **Gazebo**: Fortress or Garden (Linux recommended, integrated with ROS 2).
    *   Basic understanding of Gazebo worlds and URDF models.
3.  **Unity 3D**: Latest LTS release (Windows, macOS, or Linux).
    *   Basic familiarity with Unity Editor, scenes, and importing assets.
4.  **Python 3.10+**: With `pip` for dependency management.
5.  **Git**: For cloning the module repository.

## Installation and Setup

1.  **Clone the Repository**:
    ```bash
    git clone https://github.com/YOUR_USERNAME/Physical-AI-Humanoid-Robotics-Textbook.git
    cd Physical-AI-Humanoid-Robotics-Textbook
    git checkout 002-digital-twin-module
    ```

2.  **ROS 2 Workspace Setup (for Gazebo examples)**:
    ```bash
    # Navigate to the ROS 2 packages directory for this module
    cd module2-digital-twin/ros2_packages
    
    # Install dependencies
    rosdep install --from-paths src --ignore-src -r -y
    
    # Build the ROS 2 packages
    colcon build
    
    # Source the workspace
    source install/setup.bash
    ```

3.  **Unity Project Setup**:
    *   Open Unity Hub.
    *   Click "Add" and navigate to `module2-digital-twin/unity_projects/HumanoidScene`.
    *   Open the project in the Unity Editor. Ensure you select the correct Unity version as prompted by Unity Hub.

## Running Your First Simulation

### Gazebo (Mini-lab from Chapter 2)

1.  **Launch the Simple Humanoid Arm in Gazebo**:
    ```bash
    ros2 launch humanoid_description display_arm_launch.py
    ```
    *Expected Outcome*: A simple humanoid arm model should appear in a Gazebo simulation window, affected by gravity.

2.  **Control the Arm (Conceptual)**:
    *   You can then use ROS 2 topics or services (as described in the module's chapters) to interact with and control the joints of the simulated arm.

### Unity (Mini-lab from Chapter 3)

1.  **Open the Basic Interaction Scene**:
    *   In the Unity Editor, navigate to `Assets/Scenes/HumanoidRobotScene.unity` and open it.
    *   Ensure the humanoid robot model is loaded and visible.

2.  **Run the Unity Scene**:
    *   Press the "Play" button in the Unity Editor.
    *   *Expected Outcome*: The scene will run, and you should be able to observe the high-fidelity rendering and any pre-configured interactive elements.

## Next Steps

-   **Explore Chapters**: Dive into the detailed chapters within `module2-digital-twin/content/` for comprehensive explanations, further examples, and in-depth understanding.
-   **Experiment with Labs**: Follow the step-by-step instructions in the labs to gain practical experience.
-   **Undertake the Micro-Project**: Apply all learned concepts to complete the micro-project outlined in Chapter 5.
