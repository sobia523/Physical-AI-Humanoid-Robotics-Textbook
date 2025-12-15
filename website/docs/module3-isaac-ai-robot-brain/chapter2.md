# Chapter 2: Photorealistic Simulation & Synthetic Data

This chapter focuses on setting up NVIDIA Isaac Sim environments, generating synthetic sensor datasets for training AI models, and hands-on exercises for capturing LiDAR and RGB-D data from simulated humanoids.

## 2.1 Setting Up Isaac Sim Environments

To begin, ensure you have NVIDIA Isaac Sim installed and configured correctly. Refer to the official NVIDIA documentation for detailed installation instructions.

Once installed, you can launch Isaac Sim and load a basic environment. In this module, we will utilize custom USD (Universal Scene Description) files for our robot and environment definitions.

### 2.1.1 Loading a Humanoid Robot

Our humanoid robot model is defined in:
`module3-isaac-ai-robot-brain/isaac_sim_assets/robots/humanoid_robot.usd`

This file encapsulates the robot's geometry, articulation (joints), and sensor attachment points.

### 2.1.2 Defining a Simulation Environment

The simulation environment, including terrain and static obstacles, is defined in:
`module3-isaac-ai-robot-brain/isaac_sim_assets/environments/basic_env.usd`

### 2.1.3 Launching the Scenario

To load both the robot and the environment, we use an Isaac Sim scenario script:
`module3-isaac-ai-robot-brain/isaac_sim_assets/scenarios/sensor_capture_scenario.py`

This Python script can be executed within the Isaac Sim environment to stage the scene, initialize physics, and prepare the robot with its attached sensors.

## 2.2 Generating Synthetic Sensor Datasets

NVIDIA Isaac Sim provides powerful capabilities for generating high-fidelity synthetic sensor data. This is crucial for training robust AI models without relying solely on real-world data, which can be expensive and time-consuming to acquire.

### 2.2.1 Capturing LiDAR Data

The following ROS 2 Python node is designed to subscribe to and process synthetic LiDAR data streams published by Isaac Sim:
`module3-isaac-ai-robot-brain/ros2_packages/src/sensor_data_capture/lidar_capture_node.py`

This node can be extended to log, visualize, or further process the LiDAR point cloud or scan data.

### 2.2.2 Capturing RGB-D Camera Data

Similarly, for visual perception, we utilize a ROS 2 Python node to capture RGB and Depth images from an RGB-D camera in Isaac Sim:
`module3-isaac-ai-robot-brain/ros2_packages/src/sensor_data_capture/rgbd_capture_node.py`

This node demonstrates how to receive and potentially convert these image streams for use with computer vision algorithms.

## 2.3 Hands-on: Capture Synthetic Sensor Data

In this hands-on exercise, you will:
1.  Launch the `sensor_capture_scenario.py` in Isaac Sim.
2.  Run the `lidar_capture_node.py` and `rgbd_capture_node.py` ROS 2 nodes.
3.  Observe the data streams using ROS 2 tools (e.g., `ros2 topic echo`).
4.  (Optional) Implement basic data logging to save a few frames of sensor data.