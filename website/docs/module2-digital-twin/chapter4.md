# Chapter 4: Integrating Sensors and Perception

This chapter focuses on a critical aspect of digital twin simulation: integrating sensor data from the simulation environment with ROS 2 perception pipelines. We will explore how to synchronize Gazebo sensor outputs with ROS 2 topics, debug simulated sensor data, and implement a hands-on exercise for LiDAR-based obstacle detection.

## 4.1 Mapping Sensor Data from Simulation to Perception Pipelines

The primary goal of sensor simulation is to provide realistic data that can be consumed by the robot's perception algorithms. ROS 2 serves as the middleware that bridges the sensor data from Gazebo (and potentially Unity) to your perception nodes.

### Key ROS 2 Sensor Message Types

-   **`sensor_msgs/msg/Image`**: For camera feeds (RGB, depth, monochrome).
-   **`sensor_msgs/msg/PointCloud2`**: For LiDAR or 3D depth camera point clouds.
-   **`sensor_msgs/msg/LaserScan`**: For 2D LiDAR scans.
-   **`sensor_msgs/msg/Imu`**: For Inertial Measurement Unit data (acceleration, angular velocity, orientation).
-   **`sensor_msgs/msg/JointState`**: For reporting the state of robot joints (position, velocity, effort).

Your perception pipelines will typically subscribe to these message types, process the data, and output higher-level information (e.g., object detections, occupancy maps, robot pose estimates).

---
**Figure 4.1: Sensor Data Flow Diagram**

*This diagram illustrates the flow of sensor data from simulation to perception. It shows a simulated robot in Gazebo (or Unity) generating raw sensor data. This data is then passed through a ROS 2 bridge (`ros_gz_bridge` or custom Unity-ROS 2 connector) to be published on ROS 2 topics. A perception pipeline (represented by ROS 2 nodes, e.g., a LiDAR obstacle detector, an image processing node) subscribes to these topics, processes the data, and outputs higher-level information.*
---

## 4.2 Synchronizing Gazebo Sensors with ROS 2 Topics

The `ros_gz_bridge` package is essential for connecting Gazebo simulations to ROS 2. It translates Gazebo's native data formats into ROS 2 message types and publishes them on ROS 2 topics.

### How `ros_gz_bridge` Works

The bridge operates by specifying mappings between Gazebo's native topic names (e.g., `/lidar`, `/camera/image`) and ROS 2 topics (e.g., `/scan`, `/camera/color/image_raw`). This can be configured in a launch file or programmatically.

Example Bridge Configuration (in a ROS 2 launch file):
```python
from launch_ros.actions import Node
# ... other imports

gz_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
        '/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
        '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
        '/model/simple_humanoid/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model', # Example for joint states
    ],
    output='screen'
)
```

-   The format is `ROS_TOPIC@ROS_TYPE[GZ_TOPIC_TYPE`.
-   It's crucial to map the correct Gazebo topic names (which are often `/world/<world_name>/model/<model_name>/link/<link_name>/sensor/<sensor_name>/<data_type>`) to their ROS 2 equivalents.

---
**Figure 4.2: `ros_gz_bridge` Workflow Diagram**

*This diagram illustrates the role of `ros_gz_bridge`. On one side, Gazebo simulation publishes sensor data on its internal topics (e.g., `ignition.msgs.LaserScan`). `ros_gz_bridge` acts as an intermediary, subscribing to these Gazebo topics and converting the data into corresponding ROS 2 message types (e.g., `sensor_msgs/msg/LaserScan`). These ROS 2 messages are then published on ROS 2 topics, making them available to ROS 2 nodes within the perception pipeline.*
---

## 4.3 Debugging Simulated Sensor Outputs

Debugging sensor data is vital to ensure your perception algorithms are receiving valid input.

### Tools for Debugging

1.  **`ros2 topic list`**: Lists all active ROS 2 topics. Use this to verify your bridge is publishing data.
2.  **`ros2 topic info <topic_name>`**: Shows the message type and publishers/subscribers for a topic.
3.  **`ros2 topic echo <topic_name>`**: Prints the messages published on a topic to the console. This is useful for inspecting raw data.
4.  **`rqt_plot`**: A graphical tool for plotting numerical data from ROS 2 topics over time. Excellent for checking IMU values, joint positions, etc.
5.  **`rviz2`**: The ROS 2 visualization tool. Essential for visualizing LiDAR point clouds, camera images, TF frames, and robot models.
    -   Add `LaserScan` or `PointCloud2` displays for LiDAR.
    -   Add `Image` displays for camera feeds.
    -   Ensure your `robot_state_publisher` and `tf_static` broadcasters are correctly publishing TF frames.

---
**Figure 4.3: ROS 2 Debugging Tools Diagram**

*This diagram shows a typical ROS 2 debugging setup. `ros2 topic list` is depicted checking active topics. `ros2 topic echo` is shown inspecting message content. `rqt_plot` is graphing sensor data over time. `rviz2` is visualizing the robot model, LiDAR scans, and camera feeds in a 3D environment, allowing for visual verification of sensor outputs and robot state.*
---

### Common Debugging Steps

-   **Check `ros_gz_bridge` logs**: Ensure the bridge is running without errors and correctly mapping topics.
-   **Verify sensor data in Gazebo**: Use Gazebo's Topic Visualizer (View -> Topic Visualizer) to confirm the sensor is publishing data internally.
-   **Inspect TF Tree**: Use `ros2 run rqt_tf_tree rqt_tf_tree` to visualize the TF frames and ensure your sensor frames are correctly attached to the robot. Incorrect TF can lead to issues in perception.
-   **Check `use_sim_time`**: Ensure `use_sim_time` is set to `true` in your ROS 2 nodes if you are running in a simulation environment.

## 4.4 Hands-on: Simulate LiDAR-based Obstacle Detection

In this hands-on lab, you will extend your humanoid robot in Gazebo with a LiDAR sensor, bridge its data to ROS 2, and then implement a simple Python ROS 2 node to detect obstacles.

### Step 1: Update Humanoid URDF with LiDAR Sensor (`simple_humanoid_sensors.urdf`)

You will need to add a LiDAR sensor definition to your `simple_humanoid.urdf` file.
Navigate to `module2-digital-twin/gazebo_simulations/models/` and create or modify `simple_humanoid_sensors.urdf`.

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid_with_sensors">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Upper Arm Link -->
  <link name="upper_arm_link">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.03"/>
      </geometry>
      <origin xyz="0 0 0.15"/>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.03"/>
      </geometry>
      <origin xyz="0 0 0.15"/>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Shoulder Joint (Revolute) -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm_link"/>
    <origin xyz="0 0 0.05"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="0.5"/>
  </joint>

  <!-- LiDAR Link -->
  <link name="lidar_link">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.02"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001"/>
    </inertial>
  </link>

  <!-- LiDAR Joint (Fixed) -->
  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0 0 0.1"/>
  </joint>

  <!-- Gazebo specific settings -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>
  <gazebo reference="upper_arm_link">
    <material>Gazebo/Red</material>
  </gazebo>
  <gazebo reference="lidar_link">
    <material>Gazebo/Black</material>
    <sensor name="lidar" type="ray">
      <pose>0 0 0 0 0 0</pose>
      <visualize>true</visualize>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>640</samples>
            <resolution>1</resolution>
            <min_angle>-1.570796</min_angle>
            <max_angle>1.570796</max_angle>
          </horizontal>
          <vertical>
            <samples>1</samples>
            <resolution>1</resolution>
            <min_angle>0</min_angle>
            <max_angle>0</max_angle>
          </vertical>
        </scan>
        <range>
          <min>0.1</min>
          <max>10.0</min>
          <resolution>0.02</resolution>
        </range>
      </ray>
      <plugin name="gazebo_ros_lidar_controller" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <namespace>/demo</namespace>
          <argument>--ros-args -r ~/out:=scan</argument>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
        <frame_name>lidar_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>
  <gazebo>
    <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
      <parameters>$(find module2_ros2_packages)/config/simple_humanoid_controller.yaml</parameters>
    </plugin>
  </gazebo>
</robot>