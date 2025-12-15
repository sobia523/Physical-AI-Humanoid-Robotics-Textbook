# Chapter 4: Nav2 Path Planning for Bipedal Humanoids

This chapter guides students through setting up the Nav2 navigation stack for simulated bipedal humanoid robots, focusing on planning safe trajectories and obstacle avoidance. It includes hands-on exercises for executing a waypoint navigation mission in a simulated environment.

## 4.1 Setting Up Nav2 for Humanoids

The Nav2 stack is a powerful framework for autonomous navigation in ROS 2. Adapting it for bipedal humanoids requires careful configuration due to their unique kinematics and stability challenges.

Our Nav2 configuration is defined in a dedicated ROS 2 package:
`module3-isaac-ai-robot-brain/ros2_packages/nav2_humanoid_configs/`

### 4.1.1 Nav2 Parameters

The core behavior of Nav2 is controlled by a set of parameters. We use a custom YAML file to configure these parameters, specifically tuned for bipedal humanoid motion:
`module3-isaac-ai-robot-brain/ros2_packages/nav2_humanoid_configs/params/humanoid_nav2_params.yaml`

This file would contain critical settings such as:
*   **Robot Footprint**: Defining the robot's physical dimensions for collision avoidance.
*   **Controller Parameters**: Adjusting velocity and acceleration limits suitable for a bipedal gait.
*   **Costmap Layers**: Configuring how obstacles and inflated regions are handled in the robot's local and global maps.

### 4.1.2 Launching the Nav2 Stack

The entire Nav2 stack is launched using a Python launch file:
`module3-isaac-ai-robot-brain/ros2_packages/nav2_humanoid_configs/launch/humanoid_nav2_launch.py`

This launch file typically includes:
*   The `nav2_bringup` package to start all necessary Nav2 nodes (controller, planner, recoveries, etc.).
*   Integration with a map source (e.g., from VSLAM output as discussed in Chapter 3).
*   An optional Rviz instance for visualization.

## 4.2 Sending Navigation Goals

To command the humanoid robot to move to a specific location, we send navigation goals to the Nav2 stack. A ROS 2 node is responsible for this:
`module3-isaac-ai-robot-brain/ros2_packages/src/navigation_nodes/goal_sender_node.py`

This node utilizes the `BasicNavigator` from `nav2_simple_commander` to set an initial pose and send target `PoseStamped` messages.

## 4.3 Hands-on: Execute a Waypoint Navigation Mission

In this hands-on exercise, you will:
1.  Ensure Isaac Sim is running with your humanoid robot and connected sensor data (from Chapter 2).
2.  Launch the VSLAM pipeline (from Chapter 3) to provide a map and localization.
3.  Launch the Nav2 stack using `humanoid_nav2_launch.py`.
4.  Run the `goal_sender_node.py` to send a navigation goal to the humanoid.
5.  Observe in Rviz and Isaac Sim as the robot plans a path and attempts to navigate to the goal, avoiding any simulated obstacles.