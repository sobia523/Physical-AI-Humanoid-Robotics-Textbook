# Quickstart Guide for Module 3: The AI-Robot Brain (NVIDIA Isaac™)

This document provides step-by-step instructions for quickly setting up and verifying the core functionalities of Module 3.

## Setup Isaac Sim

(To be populated with step-by-step guide for installing and launching NVIDIA Isaac Sim.)

## Setup Isaac ROS

(To be populated with instructions for setting up the Isaac ROS development environment and required packages.)

## Clone Repository

(To be populated with instructions for cloning the textbook repository.)

## Launch Basic Isaac Sim Scenario

To launch a basic Isaac Sim scenario with a humanoid robot and confirm sensor data generation:

1.  **Start Isaac Sim**: Launch NVIDIA Isaac Sim application.
2.  **Open the Scenario Script**: Navigate to `module3-isaac-ai-robot-brain/isaac_sim_assets/scenarios/sensor_capture_scenario.py` and open it within Isaac Sim's script editor.
3.  **Run the Script**: Execute the `sensor_capture_scenario.py` script. This should load the `humanoid_robot.usd` and `basic_env.usd` (placeholders) and set up the sensors.
4.  **Launch ROS 2 Sensor Capture Nodes**: In a terminal where your ROS 2 environment is sourced, navigate to `module3-isaac-ai-robot-brain/ros2_packages/` and run:
    ```bash
    # Ensure your ROS 2 workspace is built and sourced (see ros2_packages/README.md)
    ros2 run sensor_data_capture lidar_capture_node
    ros2 run sensor_data_capture rgbd_capture_node
    ```
5.  **Verify Sensor Data**: Observe the terminal output of the `lidar_capture_node` and `rgbd_capture_node` to confirm data reception. You can also use `ros2 topic list` and `ros2 topic echo <topic_name>` to inspect the published topics.


## Run Basic VSLAM Pipeline

To launch an Isaac ROS VSLAM pipeline and visualize results in Rviz:

1.  **Ensure Isaac Sim is Running**: Start Isaac Sim and run the `sensor_capture_scenario.py` script to simulate the robot and sensors.
2.  **Build ROS 2 Workspace**: Ensure your ROS 2 workspace (`module3-isaac-ai-robot-brain/ros2_packages/`) is built and sourced.
3.  **Launch VSLAM Pipeline**: In a terminal, navigate to your ROS 2 workspace root (e.g., `ros2_ws`) and run:
    ```bash
    ros2 launch isaac_ros_vslam_configs vslam_launch.py
    ```
    This will start the VSLAM node and potentially an Rviz instance.
4.  **Launch VSLAM Visualizer (if Rviz not launched by default)**: If Rviz is not automatically launched, open a new terminal and run:
    ```bash
    ros2 run vslam_nodes vslam_visualizer_node
    ```
5.  **Verify Visualization in Rviz**:
    *   In Rviz, add `RobotModel` to visualize your robot.
    *   Add `Map` (subscribed to `/map` topic) to see the generated occupancy grid map.
    *   Add `Path` (subscribed to `/vslam/path` topic, published by `vslam_visualizer_node`) to visualize the robot's trajectory.
    *   Move the simulated robot in Isaac Sim (e.g., using teleoperation or by adjusting its pose manually) and observe the map being built and the robot's pose being estimated in Rviz.


## Run Basic Nav2 Pipeline

To launch Nav2 for a humanoid and send a navigation goal:

1.  **Ensure Isaac Sim is Running**: Start Isaac Sim and run the `sensor_capture_scenario.py` script.
2.  **Launch VSLAM Pipeline**: Follow the steps in "Run Basic VSLAM Pipeline" to launch the VSLAM system, as Nav2 will rely on its map and localization output.
3.  **Build ROS 2 Workspace**: Ensure your ROS 2 workspace (`module3-isaac-ai-robot-brain/ros2_packages/`) is built and sourced.
4.  **Launch Nav2 Stack**: In a terminal, navigate to your ROS 2 workspace root (e.g., `ros2_ws`) and run:
    ```bash
    ros2 launch nav2_humanoid_configs humanoid_nav2_launch.py
    ```
    This will start the Nav2 stack, including the planner, controller, and recovery behaviors.
5.  **Send Navigation Goal**: In a new terminal, run the goal sender node:
    ```bash
    ros2 run navigation_nodes goal_sender_node
    ```
    This node will attempt to send a predefined navigation goal to Nav2.
6.  **Verify Navigation**: Observe in Rviz (if launched) and Isaac Sim as the robot plans a path on the VSLAM-generated map and attempts to navigate to the goal, avoiding obstacles. You should see the global and local plans, and the robot's movement.

