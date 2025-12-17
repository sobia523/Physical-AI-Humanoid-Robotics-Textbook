# Quickstart Guide: Module 2 — The Digital Twin (Gazebo & Unity)

## Prerequisites

*   Ubuntu (recommended for ROS 2 and Gazebo)
*   ROS 2 (Humble/Iron) installed and configured
*   Gazebo installed (compatible with ROS 2 version)
*   Unity 3D (LTS version recommended) installed
*   ROS 2 Unity integration package (e.g., `ROS-TCP-Endpoint` for Unity) installed in Unity
*   Python 3.x installed

## Steps to Run Module Examples

This quickstart assumes you have completed the prerequisite software installations and configurations.

1.  **Clone the Repository**:
    If you haven't already, clone the entire textbook repository:
    ```bash
    git clone https://github.com/your-organization/Physical-AI-Humanoid-Robotics-Textbook.git
    cd Physical-AI-Humanoid-Robotics-Textbook
    ```

2.  **Navigate to Module 2 Directory**:
    Change into the module-specific directory:
    ```bash
    cd module2-digital-twin
    ```

3.  **ROS 2 Package Setup**:
    Build and source the ROS 2 packages within this module:
    ```bash
    colcon build --packages-select module2_digital_twin
    source install/setup.bash
    ```


4.  **Launch Gazebo Simulations**:
    To run a Gazebo simulation example:
    *   For the Chapter 2 Mini-lab (basic humanoid simulation):
        ```bash
        ros2 launch module2_digital_twin humanoid_simulation.launch.py
        ```
    *   For the Chapter 5 Micro-Project (humanoid in a room simulation):
        ```bash
        ros2 launch module2_digital_twin humanoid_room_simulation.launch.py
        ```


5.  **Run Unity Scenes**:
    To open and run Unity scenes:
    -   Open Unity Hub.
    -   Add the `unity_projects/` folder (located inside `module2-digital-twin`) as an existing project.
    -   Open the Unity project.
    -   Navigate to the relevant scene in the Project window and double-click to open it.
    -   Press the Play button in the Unity editor to run the simulation.

6.  **Verify Mini-Labs and Micro-Project**:
    Follow the instructions within the respective chapter Markdown files (e.g., `content/chapter2.md`, `content/chapter3.md`, `content/chapter5.md`) to execute mini-labs and the micro-project. Verify that the robot behavior, sensor outputs, and visual rendering match the expected outcomes described in the chapters.

## Troubleshooting

*   **ROS 2/Gazebo**: Ensure your ROS 2 environment is sourced (`source /opt/ros/<ros2-distro>/setup.bash`) and Gazebo is running correctly.
*   **Unity**: Check Unity console for errors. Ensure `ROS-TCP-Endpoint` (or similar) is correctly configured for ROS 2 communication.
*   **Dependencies**: Verify all required Python/ROS 2 dependencies are installed.
