# Quickstart Guide: AI-Robot Brain (NVIDIA Isaac)

**Branch**: `001-isaac-ai-robot-brain` | **Date**: 2025-12-12 | **Spec**: [specs/001-isaac-ai-robot-brain/spec.md](specs/001-isaac-ai-robot-brain/spec.md)
**Purpose**: This guide provides the initial steps to quickly set up the necessary environment and run a basic demonstration for the AI-Robot Brain module.

---

## 1. Prerequisites

Before you begin, ensure you have the following installed and configured:

*   **Operating System**: Ubuntu 20.04/22.04 (Linux)
*   **ROS 2**: Humble Hawksbill, Iron Irwini, or Rolling Ridley distribution.
    *   Follow the official ROS 2 installation guide for your distribution.
*   **NVIDIA Isaac Sim**:
    *   Download and install NVIDIA Omniverse Launcher.
    *   Install Isaac Sim (latest recommended version) via the Omniverse Launcher.
*   **NVIDIA Graphics Driver**: Ensure you have the latest compatible NVIDIA GPU drivers installed.

It is highly recommended that you have successfully completed **Module 1 (The Robotic Nervous System)** and **Module 2 (The Digital Twin)** as this module builds upon their foundational concepts.

## 2. Setting up the Module Workspace

1.  **Clone the Repository**:
    If you haven't already, clone the main textbook repository:
    ```bash
    git clone <repository_url>
    cd Physical-AI-Humanoid-Robotics-Textbook
    ```

2.  **Navigate to Module 3**:
    ```bash
    cd module3-ai-robot-brain
    ```

3.  **Create and Build ROS 2 Workspace**:
    Set up the ROS 2 workspace for the code examples:
    ```bash
    mkdir -p ros2_packages/src
    # Copy example packages here (details will be in specific chapters)
    # cp -r /path/to/example_package ros2_packages/src/

    # Build the workspace
    source /opt/ros/<ROS2_DISTRO>/setup.bash
    colcon build --packages-above-and-dependencies <your_package_name>
    ```

## 3. Launching Isaac Sim

1.  **Start Omniverse Launcher**:
    Launch the NVIDIA Omniverse Launcher and ensure Isaac Sim is installed.

2.  **Launch Isaac Sim**:
    Start Isaac Sim from the Launcher. This will open the 3D simulation environment.

## 4. Running a Basic VSLAM Demonstration (Conceptual)

*(Note: Detailed instructions and code will be provided in Chapter 3.)*

1.  **Load Humanoid Robot in Isaac Sim**:
    Within Isaac Sim, load the provided humanoid robot model and a simple environment (e.g., a room with some textured walls and objects).

2.  **Configure Synthetic Sensors**:
    Add and configure an RGB-D camera and IMU to the humanoid robot model within Isaac Sim. Ensure they are publishing data.

3.  **Launch Isaac ROS VSLAM Node**:
    From your ROS 2 workspace (after building the `isaac_ros_vslam` package), launch the VSLAM node:
    ```bash
    source install/setup.bash
    ros2 launch isaac_ros_vslam vslam_launch.py # Example launch file
    ```

4.  **Visualize Results in Rviz**:
    Open Rviz and configure it to visualize the `/vslam/pose` (estimated robot trajectory) and `/vslam/map_points` (built map).
    ```bash
    source /opt/ros/<ROS2_DISTRO>/setup.bash
    rviz2 -d /path/to/vslam_config.rviz # Example Rviz config
    ```

5.  **Teleoperate Robot (Optional)**:
    Move the humanoid robot within Isaac Sim (e.g., using a game controller or simple teleoperation script) and observe the VSLAM pipeline updating the pose estimation and map in Rviz.

---

This quickstart provides a high-level overview. Each chapter will delve into specific configurations, code implementations, and detailed steps for each component of the AI-Robot Brain.
