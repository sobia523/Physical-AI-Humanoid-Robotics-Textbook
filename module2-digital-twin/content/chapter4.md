# Chapter 4: Integrating Sensors and Perception

This chapter focuses on how to map simulated sensor data from Gazebo to ROS 2 topics, integrate these into basic perception pipelines, and effectively synchronize and debug the data flow between simulation and ROS 2.

## Mapping Sensor Data from Simulation to Perception Pipelines

Once sensors are simulated in Gazebo, they typically publish data to specific ROS 2 topics. Your ROS 2 nodes can then subscribe to these topics to receive the data.

### Common Sensor Data Types

*   **LiDAR**: Publishes `sensor_msgs/LaserScan` messages.
*   **IMU**: Publishes `sensor_msgs/Imu` messages.
*   **Depth Camera**: Publishes `sensor_msgs/Image` (for RGB and depth) and `sensor_msgs/PointCloud2` (for point clouds).

## Synchronizing Gazebo Sensors with ROS 2 Topics

Ensuring that sensor data from Gazebo is correctly synchronized and published to ROS 2 topics is crucial for any perception or control pipeline. Gazebo's ROS 2 plugins handle this mapping.

### Key Configuration Points

*   **Plugin Configuration**: Within your URDF or SDF, sensor plugins specify the ROS 2 topic names, frame names, and update rates.
*   **ROS 2 Launch Files**: Use ROS 2 launch files (`.launch.py`) to start Gazebo, load your robot, and launch any necessary ROS 2 nodes.

## Debugging Simulated Sensor Outputs

Debugging sensor data is a critical skill. Tools like `ros2 topic echo`, `rviz2`, and custom Python scripts can help verify the integrity and correctness of simulated sensor outputs.

### Steps for Debugging

1.  **Check Topic Publication**: Use `ros2 topic list` to see if your sensor topics are being published.
2.  **Echo Topic Data**: Use `ros2 topic echo <topic_name>` to view raw sensor messages.
3.  **Visualize Data**: Use `rviz2` to visualize `LaserScan` data, `Image` data, or `PointCloud2` data.
4.  **Inspect Timestamps**: Ensure sensor data timestamps are consistent and reasonable, especially when dealing with multiple sensors.

## Hands-on: Simulate LiDAR-based Obstacle Detection

In this hands-on activity, you will simulate LiDAR-based obstacle detection.

### Prerequisites

- You have a working ROS 2 and Gazebo installation.
- You have built the ROS 2 packages in this module by running `colcon build` from the `Physical-AI-Humanoid-Robotics-Textbook` directory.
- You have sourced the ROS 2 workspace: `source install/setup.bash`.

### Steps

1.  **Open a new terminal** and launch the Gazebo simulation with the humanoid robot:

    ```bash
    ros2 launch module2_digital_twin humanoid_simulation.launch.py
    ```

2.  **Open a second terminal** and run the LiDAR processor node:

    ```bash
    ros2 run module2_digital_twin lidar_processor
    ```

3.  **Observe the Output**: The `lidar_processor` node will subscribe to the `/scan` topic from the simulated LiDAR sensor. It will process the data and print a message to the console if an obstacle is detected within a certain range.

4.  **Visualize in RViz2 (Optional)**:
    -   Open a third terminal and launch RViz2: `rviz2`
    -   In RViz2, click "Add" and select "By topic".
    -   Add the `/scan` topic of type `LaserScan`.
    -   You should see the LiDAR scan data visualized in the RViz2 window.

5.  **Close the Simulation and Nodes**: To stop the simulation and nodes, press `Ctrl+C` in each terminal.

### Expected Outcome

- The Gazebo simulation runs with the humanoid robot.
- The `lidar_processor` node starts and subscribes to the `/scan` topic.
- When the robot is near an obstacle, the `lidar_processor` node prints an "Obstacle detected!" message.
- (Optional) You can visualize the LiDAR scan data in RViz2.

