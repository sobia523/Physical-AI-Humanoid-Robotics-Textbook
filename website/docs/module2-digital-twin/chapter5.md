# Chapter 5: Micro-Project: Simulated Humanoid Environment

This chapter culminates your learning in Module 2 by guiding you through a micro-project: building a comprehensive simulated humanoid environment. You will integrate the physics, sensor simulation, and ROS 2 communication concepts learned in previous chapters to create a digital twin of a small room, deploy a humanoid robot within it, and test basic navigation and object interaction.

## 5.1 Build a Digital Twin of a Small Room

Creating a simulated environment involves defining the geometry, materials, and static elements of the room. This will be done using a Gazebo `.world` file.

### Designing the Room

Consider a simple room with four walls, a floor, and perhaps a few static obstacles (e.g., a table, a chair).

-   **Walls and Floor**: Use basic primitive shapes (boxes) for walls and a plane for the floor. Assign simple materials to them.
-   **Static Obstacles**: Add objects that the robot will need to navigate around or interact with.
-   **Lighting**: Ensure the room is adequately lit using Gazebo's light sources.

### Example `small_room.world` Structure

Navigate to `module2-digital-twin/gazebo_simulations/worlds/` and create `small_room.world`.

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="small_room_world">
    <include>
      <uri>model://sun</uri>
    </include>
    <light name='room_light' type='point'>
      <pose>0 0 3 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <attenuation>
        <range>10</range>
        <constant>0.2</constant>
        <linear>0.05</linear>
        <quadratic>0.01</quadratic>
      </attenuation>
      <cast_shadows>false</cast_shadows>
    </light>
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <physics name="ode_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>50</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0</cfm>
          <erp>0.2</erp>
        </constraints>
      </ode>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Floor -->
    <model name="floor">
      <static>true</static>
      <link name="floor_link">
        <collision name="collision">
          <geometry><box><size>5 5 0.1</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>5 5 0.1</size></box></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script></material>
        </visual>
        <pose>0 0 -0.05 0 0 0</pose>
      </link>
    </model>

    <!-- Walls (Example) -->
    <model name="wall_x_pos">
      <static>true</static>
      <link name="wall_x_pos_link">
        <collision name="collision">
          <geometry><box><size>0.1 5 2.5</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.1 5 2.5</size></box></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Bricks</name></script></material>
        </visual>
        <pose>2.5 0 1.25 0 0 0</pose>
      </link>
    </model>
    <model name="wall_x_neg">
      <static>true</static>
      <link name="wall_x_neg_link">
        <collision name="collision">
          <geometry><box><size>0.1 5 2.5</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.1 5 2.5</size></box></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Bricks</name></script></material>
        </visual>
        <pose>-2.5 0 1.25 0 0 0</pose>
      </link>
    </model>
    <!-- Add Y walls similarly -->

    <!-- Static Obstacle (Example: table) -->
    <model name="table">
      <static>true</static>
      <link name="table_link">
        <collision name="collision">
          <geometry><box><size>1 0.6 0.75</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>1 0.6 0.75</size></box></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Wood</name></script></material>
        </visual>
        <pose>1 1 0.375 0 0 0</pose>
      </link>
    </model>

  </world>
</sdf>

---
**Figure 5.1: Small Room Environment Diagram**

*This diagram illustrates a top-down view of the `small_room.world` in Gazebo. It shows a rectangular room defined by four walls (labeled North, South, East, West) and a flat floor. A simulated humanoid robot is positioned near the center. Various static obstacles, such as a table and a chair, are distributed within the room, indicating the environment the robot needs to navigate.*
---

## 5.2 Integrate Humanoid Robot with Physics, Sensors, and Basic Control

Now, you will integrate your sensor-equipped humanoid robot (from Chapter 4) into this new room environment. This involves ensuring the robot's URDF is correctly loaded and its ROS 2 controllers and sensor bridges are active.

### Integration Steps

1.  **Launch File Modification**: You will modify your launch file (`humanoid_sensors_gz.launch.py`) to load the `small_room.world` instead of the `basic_humanoid_world.world`.
2.  **Robot Placement**: Adjust the initial pose of the robot within the world to ensure it's placed appropriately.
3.  **Controller Activation**: Ensure the `gazebo_ros2_control` plugin and your joint controllers are correctly loaded and communicate with the robot.
4.  **Sensor Activation**: Verify that the LiDAR, IMU, and depth camera sensors are active and their data is being bridged to ROS 2 topics.

## 5.3 Test Simple Navigation and Object Interaction in Simulation

This section focuses on developing basic capabilities for the humanoid robot within the simulated room.

### Simple Navigation

Navigation for humanoid robots in complex environments is a challenging field (covered in Module 3). For this micro-project, we will focus on *simple* navigation tasks, such as:

-   **Waypoint Following**: Guiding the robot to a series of predefined points in the room.
-   **Obstacle Avoidance**: Using LiDAR data (from Chapter 4's `lidar_obstacle_detector`) to perform reactive obstacle avoidance.
-   **Teleoperation**: Manual control of the robot's base or limbs to move it through the environment.

---
**Figure 5.2: Micro-Project Navigation Flow Diagram**

*This diagram illustrates the navigation flow for the humanoid robot in the simulated room. It shows the robot's LiDAR sensor detecting obstacles, feeding this data to a ROS 2 obstacle avoidance node (e.g., `lidar_obstacle_detector`), which then influences the robot's movement commands published to `/cmd_vel` by the `simple_navigator` node. Arrows indicate the data flow from sensor to perception to control, enabling the robot to navigate the environment while avoiding collisions.*
---

### Object Interaction

Interaction tasks for humanoid robots often involve grasping and manipulation. For this micro-project, we will define *basic* object interaction, such as:

-   **Reaching a Target**: Moving a specific limb to a designated point in the environment.
-   **Pushing an Object**: Applying a force to a simulated object to move it.

You will implement simple ROS 2 Python nodes (e.g., `simple_navigator.py`, `object_manipulator.py`) to achieve these tasks.

---
**Figure 5.3: Robot-Object Interaction Diagram**

*This diagram depicts a humanoid robot interacting with a static object (e.g., a cube) in the simulated room. It shows the robot's arm extended towards the object, illustrating a "push" action. The diagram indicates a ROS 2 node (e.g., `object_manipulator`) sending commands (e.g., arm velocity, force application) to the robot's joints, resulting in physical interaction with the object within Gazebo.*
---

## 5.4 Include a Lab Report with Screenshots, Sensor Plots, and Observations

The micro-project culminates in a lab report documenting your work. This report should demonstrate your understanding and the functionality of your simulated environment.

### Report Requirements

1.  **Introduction**: Briefly describe the goal of the micro-project and the setup.
2.  **Environment Description**: 
    -   Screenshot of your `small_room.world` in Gazebo, highlighting walls, obstacles, and the humanoid robot.
    -   Description of the `.world` file components.
3.  **Robot Integration**: 
    -   Screenshot of the sensor-equipped humanoid robot in Gazebo.
    -   Explanation of how the URDF was integrated and what sensors are active.
4.  **Navigation Task**: 
    -   Description of the navigation task performed (e.g., "robot navigates from point A to point B avoiding obstacle C").
    -   Screenshot(s) demonstrating the robot successfully performing the navigation.
    -   Sensor plots (e.g., LiDAR scan data from `rqt_plot` during obstacle avoidance).
    -   Observations on robot behavior and challenges encountered.
5.  **Object Interaction Task**: 
    -   Description of the object interaction task performed (e.g., "robot pushes a box to a target area").
    -   Screenshot(s) demonstrating the robot successfully performing the interaction.
    -   Observations on interaction dynamics and any limitations.
6.  **Conclusion**: Summarize your findings, reflect on the challenges, and suggest future improvements.

This micro-project is designed to consolidate your understanding of digital twins, Gazebo, Unity, and ROS 2 integration, preparing you for more advanced topics in subsequent modules.

```

## 5.2 Integrate Humanoid Robot with Physics, Sensors, and Basic Control

Now, you will integrate your sensor-equipped humanoid robot (from Chapter 4) into this new room environment. This involves ensuring the robot's URDF is correctly loaded and its ROS 2 controllers and sensor bridges are active.

### Integration Steps

1.  **Launch File Modification**: You will modify your launch file (`humanoid_sensors_gz.launch.py`) to load the `small_room.world` instead of the `basic_humanoid_world.world`.
2.  **Robot Placement**: Adjust the initial pose of the robot within the world to ensure it's placed appropriately.
3.  **Controller Activation**: Ensure the `gazebo_ros2_control` plugin and your joint controllers are correctly loaded and communicate with the robot.
4.  **Sensor Activation**: Verify that the LiDAR, IMU, and depth camera sensors are active and their data is being bridged to ROS 2 topics.

## 5.3 Test Simple Navigation and Object Interaction in Simulation

This section focuses on developing basic capabilities for the humanoid robot within the simulated room.

### Simple Navigation

Navigation for humanoid robots in complex environments is a challenging field (covered in Module 3). For this micro-project, we will focus on *simple* navigation tasks, such as:

-   **Waypoint Following**: Guiding the robot to a series of predefined points in the room.
-   **Obstacle Avoidance**: Using LiDAR data (from Chapter 4's `lidar_obstacle_detector`) to perform reactive obstacle avoidance.
-   **Teleoperation**: Manual control of the robot's base or limbs to move it through the environment.

### Object Interaction

Interaction tasks for humanoid robots often involve grasping and manipulation. For this micro-project, we will define *basic* object interaction, such as:

-   **Reaching a Target**: Moving a specific limb to a designated point in the environment.
-   **Pushing an Object**: Applying a force to a simulated object to move it.

You will implement simple ROS 2 Python nodes (e.g., `simple_navigator.py`, `object_manipulator.py`) to achieve these tasks.

## 5.4 Include a Lab Report with Screenshots, Sensor Plots, and Observations

The micro-project culminates in a lab report documenting your work. This report should demonstrate your understanding and the functionality of your simulated environment.

### Report Requirements

1.  **Introduction**: Briefly describe the goal of the micro-project and the setup.
2.  **Environment Description**:
    -   Screenshot of your `small_room.world` in Gazebo, highlighting walls, obstacles, and the humanoid robot.
    -   Description of the `.world` file components.
3.  **Robot Integration**:
    -   Screenshot of the sensor-equipped humanoid robot in Gazebo.
    -   Explanation of how the URDF was integrated and what sensors are active.
4.  **Navigation Task**:
    -   Description of the navigation task performed (e.g., "robot navigates from point A to point B avoiding obstacle C").
    -   Screenshot(s) demonstrating the robot successfully performing the navigation.
    -   Sensor plots (e.g., LiDAR scan data from `rqt_plot` during obstacle avoidance).
    -   Observations on robot behavior and challenges encountered.
5.  **Object Interaction Task**:
    -   Description of the object interaction task performed (e.g., "robot pushes a box to a target area").
    -   Screenshot(s) demonstrating the robot successfully performing the interaction.
    -   Observations on interaction dynamics and any limitations.
6.  **Conclusion**: Summarize your findings, reflect on the challenges, and suggest future improvements.

This micro-project is designed to consolidate your understanding of digital twins, Gazebo, Unity, and ROS 2 integration, preparing you for more advanced topics in subsequent modules.