# Quickstart: ROS 2 Humanoid Control Module

**Branch**: `001-ros2-humanoid-control` | **Date**: 2025-12-11 | **Spec**: `specs/001-ros2-humanoid-control/spec.md`
**Input**: Feature specification and implementation plan for the ROS 2 Humanoid Control Module.

## Overview

This Quickstart guide provides instructions to quickly set up the development environment and run the code examples provided within the "ROS 2 Humanoid Control Module." The module primarily uses ROS 2 with Python (`rclpy`) for robot control and URDF for robot modeling, designed to be reproducible in simulation environments.

## Prerequisites

Before proceeding, ensure you have the following installed:

1.  **Ubuntu 22.04 LTS (Jammy Jellyfish)**: The recommended operating system for ROS 2 development.
2.  **ROS 2 Humble Hawksbill (or Iron Irwini)**: Follow the official ROS 2 documentation for installation instructions:
    *   [ROS 2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation.html)
    *   [ROS 2 Iron Installation Guide](https://docs.ros.org/en/iron/Installation.html)
    *   Ensure you install the `ros-humble-desktop` (or `ros-iron-desktop`) package, which includes `rclpy`, `rviz`, `gazebo`, and other essential tools.
3.  **Python 3.10+**: Typically comes pre-installed with Ubuntu 22.04 and is required for `rclpy`.
4.  **Colcon**: The ROS 2 build tool, usually installed with the desktop version of ROS 2.
5.  **Git**: For cloning the repository containing the module's code examples.

## Setup Instructions

Follow these steps to get the module's code examples running on your system:

### 1. Clone the Repository

Open a terminal and clone the main textbook repository:

```bash
git clone https://github.com/your-organization/Physical-AI-Humanoid-Robotics-Textbook.git
cd Physical-AI-Humanoid-Robotics-Textbook
```

### 2. Navigate to the Module Directory

Change to the directory for this specific module:

```bash
cd module1-ros2-humanoid-control
```

### 3. Initialize ROS 2 Workspace and Install Dependencies

This module is structured as a ROS 2 workspace. Source your ROS 2 installation and then install any Python dependencies:

```bash
# Source your ROS 2 installation (replace 'humble' with your ROS 2 distro if different)
source /opt/ros/humble/setup.bash

# Install Python dependencies (if any specific to the examples)
# Example: If examples use additional Python packages not in rclpy
# pip install -r requirements.txt
```

### 4. Build the ROS 2 Packages

Use `colcon build` to build the ROS 2 packages within this module. This will compile any C++ nodes and install Python packages into the workspace `install` space.

```bash
colcon build --symlink-install
```

### 5. Source the Workspace

After building, you need to source the workspace to make the new ROS 2 packages available to your environment:

```bash
source install/setup.bash
```

### 6. Run Examples

Each chapter will contain specific instructions on how to run its associated code examples. Generally, this will involve launching ROS 2 nodes or starting simulation environments.

**Example: Running a basic publisher-subscriber (Chapter 2)**

```bash
# In one terminal, run the publisher
ros2 run <package_name> simple_publisher

# In another terminal, run the subscriber
ros2 run <package_name> simple_subscriber
```

**Example: Launching a URDF humanoid in Gazebo (Chapter 5)**

```bash
ros2 launch <urdf_package_name> display.launch.py # To view in Rviz
ros2 launch <urdf_package_name> gazebo.launch.py # To launch in Gazebo
```

Refer to the individual chapter sections within the module for detailed execution steps for each example.
