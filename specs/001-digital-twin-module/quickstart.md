# Quickstart Guide: Module 2 — The Digital Twin (Gazebo & Unity)

**Branch**: `001-digital-twin-module` | **Date**: 2025-12-11
**Context**: A guide to quickly set up the development environment and run the initial examples for the 'Digital Twin Module'.

## 1. Prerequisites Installation

Ensure you have the following software installed on your system. Refer to official documentation for detailed installation instructions.

-   **ROS 2 (Humble/Iron)**: Install a compatible ROS 2 distribution for your Linux system.
    -   [ROS 2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation.html)
    -   [ROS 2 Iron Installation Guide](https://docs.ros.org/en/iron/Installation.html)
-   **Gazebo**: Gazebo (formerly Gazebo Classic) is typically installed with ROS 2. Verify its installation.
    -   [Gazebo Documentation](https://gazebosim.org/docs)
-   **Unity 3D**: Install Unity Hub and Unity Editor (version 2022.3 LTS or newer recommended for compatibility with ROS-Unity integrations).
    -   [Unity Download Archive](https://unity.com/download/archive)

## 2. Clone the Repository

Navigate to your desired development directory and clone the project repository:

```bash
git clone https://github.com/your-username/Physical-AI-Humanoid-Robotics-Textbook.git
cd Physical-AI-Humanoid-Robotics-Textbook
```

## 3. Setup ROS 2 Workspace

This module includes ROS 2 packages that need to be built.

```bash
# Navigate to the module's ROS 2 workspace (adjust path as necessary)
cd module2-digital-twin/ros2_packages

# Install dependencies (if any)
rosdep install --from-paths src --ignore-src -r -y

# Build the ROS 2 packages
colcon build

# Source the workspace
# For bash/zsh:
source install/setup.bash
# For PowerShell:
# . install/setup.ps1
```

## 4. Open the Unity Project

Open the Unity project associated with this module:

1.  Launch **Unity Hub**.
2.  Click "Add" and navigate to the `Physical-AI-Humanoid-Robotics-Textbook/module2-digital-twin/unity_projects/HumanoidScene` directory.
3.  Select the folder and click "Add Project".
4.  Open the project in Unity Editor.

## 5. Run a Simple Gazebo Simulation

After sourcing your ROS 2 workspace (Step 3), you can run an introductory Gazebo simulation:

```bash
# Example: Launch a simple humanoid arm in Gazebo
ros2 launch module2_description display_arm_launch.py
```
*(Note: The exact launch file name may vary, refer to chapter content for specifics.)*

## 6. Run a Simple Unity Scene

Once the Unity project is open (Step 4):

1.  In the Unity Editor, navigate to the `Assets/Scenes` folder.
2.  Open the `SampleScene` (or equivalent introductory scene).
3.  Press the "Play" button in the Unity Editor to run the scene.

This will get you started with the foundational examples of the Digital Twin module. Refer to individual chapter guides for more detailed instructions and exercises.