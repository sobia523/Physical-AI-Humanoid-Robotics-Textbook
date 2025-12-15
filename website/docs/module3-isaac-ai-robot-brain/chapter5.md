# Chapter 5: Micro-Project: Autonomous Navigation Pipeline

This chapter focuses on a micro-project where students combine perception (VSLAM) and planning (Nav2) to implement an end-to-end autonomous navigation pipeline. It includes testing a humanoid robot navigating around obstacles and requiring a lab report with maps, sensor data plots, and trajectory visualization.

## 5.1 Integrating VSLAM and Nav2

The autonomous navigation pipeline brings together the capabilities developed in previous chapters:
*   **Perception (VSLAM)**: From Chapter 3, providing real-time localization and mapping (`/vslam/map`, `/vslam/pose`).
*   **Planning (Nav2)**: From Chapter 4, providing path planning and execution capabilities.

The integration of these components is orchestrated by a combined ROS 2 launch file:
`module3-isaac-ai-robot-brain/ros2_packages/launch/autonomous_navigation_pipeline.launch.py`

This launch file starts both the VSLAM and Nav2 stacks, ensuring they are properly connected via ROS 2 topics.

## 5.2 Testing Autonomous Navigation Scenarios

To test the end-to-end pipeline, we use a dedicated Python script:
`module3-isaac-ai-robot-brain/ros2_packages/src/micro_project/autonomous_mission.py`

This script defines a sequence of waypoints for the humanoid robot to follow, allowing us to evaluate its ability to:
*   Navigate to specified goals.
*   Dynamically avoid obstacles.
*   Maintain localization within the environment.

## 5.3 Lab Report: Maps, Sensor Data, and Trajectory Visualization

A critical part of this micro-project is to generate a lab report documenting the robot's performance. This report should include:
*   **Generated Maps**: Visualizations of the environment map created by the VSLAM system.
*   **Sensor Data Plots**: Examples of captured LiDAR and RGB-D data (e.g., point cloud snapshots, image frames).
*   **Trajectory Visualization**: Rviz screenshots or recorded videos showing the robot's planned paths and actual executed trajectories within the simulated environment.

## 5.4 Hands-on: Autonomous Navigation Micro-Project

In this comprehensive hands-on exercise, you will:
1.  Launch Isaac Sim with your humanoid robot and an environment containing obstacles.
2.  Launch the full autonomous navigation pipeline using `autonomous_navigation_pipeline.launch.py`.
3.  Execute the `autonomous_mission.py` script to send the waypoint mission to the robot.
4.  Monitor the robot's progress in Isaac Sim and Rviz, paying close attention to its localization accuracy, path planning, and obstacle avoidance behavior.
5.  Gather data and visualizations to compile your lab report.