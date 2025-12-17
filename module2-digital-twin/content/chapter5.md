# Chapter 5: Micro-Project: Simulated Humanoid Environment

This chapter guides you through a micro-project where you will apply all the knowledge gained in this module to build a functional digital twin of a small room with an integrated humanoid robot. This project will demonstrate practical skills in physics, sensors, and basic control within a simulated environment.

## Project Overview

The goal is to create a complete simulated environment where a humanoid robot can perform simple tasks like navigation and object interaction. You will integrate the Gazebo world, the humanoid robot model, sensor simulation, and ROS 2 control.

## Building a Digital Twin of a Small Room

### Steps

1.  **Design the Room**: Create simple 3D models for walls, a floor, and basic furniture (e.g., a table, a cube) in a CAD software or use existing Gazebo models.
2.  **Create a Gazebo World**: Combine these models into a new Gazebo `.world` file. Ensure proper collision properties for all objects.
3.  **Place the Humanoid**: Integrate your humanoid robot model (from Chapter 2) into this new Gazebo world.

## Integrate Humanoid Robot with Physics, Sensors, and Basic Control

### Steps

1.  **Physics**: Verify that gravity, collisions, and robot dynamics are correctly simulated within your custom room environment.
2.  **Sensors**: Ensure your humanoid robot's sensors (LiDAR, IMU, depth camera) are functioning correctly within the new environment and publishing data to ROS 2 topics.
3.  **Basic Control**: Implement simple ROS 2 nodes to provide basic control commands to the humanoid robot (e.g., joint position commands, velocity commands) to enable basic navigation or interaction.

## Test Simple Navigation and Object Interaction in Simulation

### Steps

1.  **Navigation**: Guide the humanoid robot to navigate around obstacles in the simulated room.
2.  **Object Interaction**: Implement a simple interaction task, such as pushing a light object or detecting proximity to a specific item.

## Running the Micro-Project

This section provides the steps to run the integrated micro-project.

### Prerequisites

- You have a working ROS 2 and Gazebo installation.
- You have built the ROS 2 packages in this module by running `colcon build` from the `Physical-AI-Humanoid-Robotics-Textbook` directory.
- You have sourced the ROS 2 workspace: `source install/setup.bash`.

### Steps

1.  **Open a new terminal** and launch the Gazebo simulation for the micro-project:

    ```bash
    ros2 launch module2_digital_twin humanoid_room_simulation.launch.py
    ```

2.  **Observe the Simulation**: A Gazebo window will open, showing the humanoid robot in a small room with some obstacles.

3.  **Open a second terminal** and run the simple navigator node:

    ```bash
    ros2 run module2_digital_twin simple_navigator
    ```

4.  **Observe the Robot's Behavior**: The `simple_navigator` node will use the LiDAR data to perform basic obstacle avoidance. The robot will attempt to move forward and turn when it detects an obstacle.

5.  **Visualize in RViz2 (Optional)**:
    -   Open a third terminal and launch RViz2: `rviz2`
    -   In RViz2, add the `/scan` topic to visualize the LiDAR data and the `/odom` topic to see the robot's estimated position.

6.  **Close the Simulation and Nodes**: To stop the simulation and nodes, press `Ctrl+C` in each terminal.

### Expected Outcome

- The Gazebo simulation runs with the humanoid robot in the small room.
- The `simple_navigator` node enables the robot to perform basic obstacle avoidance.
- The robot moves forward and turns when it gets close to a wall or obstacle.
- (Optional) You can visualize the robot's sensor data and odometry in RViz2.

## Lab Report with Screenshots, Sensor Plots, and Observations


A crucial part of this micro-project is documenting your work. Below is a template for your lab report.

### Lab Report Template

```markdown
# Micro-Project Lab Report: Simulated Humanoid Environment

**Author**: [Your Name]
**Date**: [Date of Submission]

## 1. Project Goal

*State the objective of your micro-project here. What were you trying to achieve?*

## 2. Methodology

*Briefly describe the steps you took to build the simulation.*
-   **Gazebo World**: How did you create the room and obstacles?
-   **Robot Integration**: How did you add the humanoid to the world?
-   **Control Logic**: What approach did your `simple_navigator` node take?

## 3. Simulation Setup

*Include screenshots of your simulation environment.*

**(Insert Screenshot of Gazebo World with Robot)**
*Caption: A screenshot of the Gazebo simulation environment, showing the humanoid robot in the custom-built room.*

**(Insert Screenshot of RViz2 Visualization - Optional)**
*Caption: A screenshot of RViz2 visualizing the LiDAR scans and robot odometry.*

## 4. Robot Behavior

*Describe the robot's navigation and interaction capabilities. How did it behave in the simulation?*

## 5. Sensor Data Analysis

*Include plots or samples of sensor data.*

**(Insert Plot of LiDAR Data)**
*Caption: A plot showing a sample LiDAR scan, highlighting detected obstacles.*

**(Insert Plot of Odometry Data - Optional)**
*Caption: A plot showing the robot's estimated path from odometry data.*

## 6. Observations and Results

*Discuss the success of your project. Did it meet the goals? What challenges did you face? How could you improve it?*

## 7. Conclusion

*Summarize what you learned from this micro-project and how it demonstrates your understanding of digital twins.*
```


## Conclusion

The micro-project serves as a hands-on demonstration of building and integrating a digital twin for humanoid robots. It reinforces concepts from all previous chapters, providing a practical foundation for more advanced robotics applications.
