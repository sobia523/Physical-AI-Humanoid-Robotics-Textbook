# Chapter 5: URDF for Humanoids and Simulation

## 5.1 Understanding URDF (Unified Robot Description Format)

The Unified Robot Description Format (URDF) is an XML file format used in ROS to describe all aspects of a robot. It's a foundational tool for anyone working with robotics, as it allows for a standardized way to represent a robot's kinematic and dynamic properties, its visual appearance, and its collision behavior. For humanoid robots, URDF is essential for defining their complex structure and enabling realistic simulation and control.

The XML structure of URDF is hierarchical, building the robot from individual components:

-   **Links**: These are the rigid bodies of the robot. Think of them as the physical segments of the robot, such as a forearm, a bicep, or a torso. Each link has a mass, inertia, and visual/collision properties.
    ```xml
    <link name="base_link">
      <visual>
        <geometry><box size="0.1 0.1 0.1"/></geometry>
      </visual>
      <collision>
        <geometry><box size="0.1 0.1 0.1"/></geometry>
      </collision>
      <inertial>
        <mass value="0.1"/>
        <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
      </inertial>
    </link>
    ```

-   **Joints**: Joints define the connections between two links, specifying their relative motion. Joints introduce degrees of freedom (DOF) to the robot. Common types of joints include:
    -   **Fixed**: No relative motion; rigidly connects two links.
    -   **Revolute**: Allows rotation around a single axis (e.g., a hinge joint like an elbow). This is a common joint type for humanoid limbs.
    -   **Prismatic**: Allows translation along a single axis (e.g., a linear actuator).
    Each joint specifies its `parent` and `child` links, its `origin` (position and orientation relative to the parent), and its `axis` of motion.
    ```xml
    <joint name="shoulder_pitch_joint" type="revolute">
      <parent link="base_link"/>
      <child link="upper_arm_link"/>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-1.57" upper="1.57" effort="10" velocity="0.5"/>
    </joint>
    ```

-   **Origin**: Defines the position and orientation of a child frame relative to its parent frame, or the pose of a visual/collision element relative to its link. It uses `xyz` for translation and `rpy` (roll, pitch, yaw) for rotation.

-   **Visual Properties**: The `<visual>` tag within a link defines how the link appears in visualization tools like Rviz. This includes the geometry (box, cylinder, sphere, mesh) and material properties (color, texture). These are for aesthetic purposes and do not affect physics.

-   **Collision Properties**: The `<collision>` tag within a link defines the geometry used for collision detection in simulation environments. This is crucial for preventing the robot from self-colliding or colliding with its environment. Often, simplified geometries are used for collisions to reduce computational overhead.

-   **Inertial Properties**: The `<inertial>` tag specifies the mass and inertia tensor of a link. These properties are vital for accurate physics simulation, allowing simulators to correctly calculate how forces and torques will affect the robot's motion.

## 5.2 Building Simple URDF Models

Creating a URDF model involves defining each link and joint, specifying their properties and hierarchical relationships. For a humanoid limb, such as a 2-DOF arm, you would typically define links for the shoulder, upper arm, forearm, and hand, connected by revolute joints that allow rotation.

-   **Defining Geometry, Materials, and Textures**:
    -   **Geometry**: Links can be represented by primitive shapes (boxes, cylinders, spheres) or complex meshes (e.g., STL files). Meshes provide detailed visual representations but can be computationally intensive.
    -   **Materials**: Defined using RGB colors or by referencing external textures, giving the robot a visually distinct appearance in simulation.

-   **Incorporating Joint Limits and Dynamics**:
    -   **Joint Limits**: For revolute and prismatic joints, `<limit>` tags define the `lower` and `upper` bounds of motion, as well as the maximum `velocity` and `effort` (torque/force) the joint can exert. These are crucial for realistic robot behavior and preventing impossible movements.
    -   **Dynamics**: The `<dynamics>` tag (less commonly used for basic models) can specify friction and damping coefficients for a joint, affecting how it moves under force.

## 5.3 Integrating URDF Models with ROS 2 Simulation

Once a URDF model is defined, it needs to be integrated into ROS 2 to be visualized and simulated.

-   **Displaying URDF models in `Rviz`**:
    `Rviz` (ROS Visualization) is a powerful 3D visualizer for ROS. It can display URDF models, sensor data, and other ROS messages. To view a URDF model, you typically need:
    1.  The URDF file itself.
    2.  A `robot_state_publisher` node, which reads the URDF and the joint states, then publishes the robot's kinematics (transformations between links) as `tf2` messages.
    3.  A `joint_state_publisher` node (or a custom node publishing joint states), which provides the current positions of the joints.

-   **Publishing Joint States using `robot_state_publisher` and `joint_state_publisher`**:
    -   `joint_state_publisher`: A GUI or a simple node that allows manual manipulation or publishes fixed joint states, respectively. It publishes `sensor_msgs/msg/JointState` messages.
    -   `robot_state_publisher`: Subscribes to `sensor_msgs/msg/JointState` messages and the robot's URDF, then computes and publishes the `tf2` frames for all links in the robot. This allows Rviz to display the robot in motion.

-   **Introduction to `Gazebo` for Physics Simulation (Conceptual)**:
    While Rviz is excellent for visualization, `Gazebo` is a popular 3D robotics simulator that provides a robust physics engine. It allows for realistic simulation of robot dynamics, sensor feedback, and interactions with the environment. Integrating URDF with Gazebo involves extending the URDF with Gazebo-specific tags (e.g., for plugins, sensors) and launching the model within a Gazebo world. This will be covered conceptually here and in more detail in later modules.

## 5.4 Example: Humanoid Limb Model in Rviz

This section will walk through creating a basic URDF file for a simple humanoid limb (e.g., a two-link arm) and then launching it in Rviz. We will demonstrate how to set up the necessary ROS 2 nodes to visualize the arm and manually control its joint positions.

### Running the Example

To run the URDF visualization example, you will first need to build your ROS 2 workspace. Navigate to the root of your `module1-ros2-humanoid-control` directory and execute:

```bash
colcon build --symlink-install
```

After a successful build, source your workspace to make the new packages available:

```bash
source install/setup.bash
```

Now, you can launch the URDF model in Rviz using the launch file we created:

```bash
ros2 launch module1_ros2_humanoid_control display_arm_launch.py
```

This command will start `robot_state_publisher`, `rviz2`, and `joint_state_publisher_gui`. In the `joint_state_publisher_gui`, you can use the sliders to manipulate the `shoulder_pitch_joint` and `elbow_yaw_joint`. As you move the sliders, you should see the `simple_humanoid_arm` model in Rviz animate accordingly.

Alternatively, if you want to programmatically publish joint states, you can run the `simple_joint_state_publisher.py` node instead of `joint_state_publisher_gui`. In that case, you would modify the `display_arm_launch.py` to launch your custom publisher node.

To run the custom joint state publisher instead of the GUI:

1.  **Stop** any running `display_arm_launch.py`.
2.  **Modify `display_arm_launch.py`**:
    *   Comment out or remove the `joint_state_publisher_gui` node.
    *   Add a node for `simple_joint_state_publisher.py`:
        ```python
        Node(
            package='module1_ros2_humanoid_control',
            executable='joint_state_publisher.py',
            name='simple_joint_state_publisher_node',
            output='screen',
            emulate_tty=True,
        )
        ```
3.  **Rerun the launch file**:
    ```bash
    ros2 launch module1_ros2_humanoid_control display_arm_launch.py
    ```

You should now see the arm model in Rviz moving automatically based on the sine wave animation from `simple_joint_state_publisher.py`.

## 5.5 Best Practices for URDF Modeling

Creating effective URDF models requires attention to detail and adherence to best practices.

-   **Modularity and Reusability**: Break down complex robots into smaller, reusable URDF components (e.g., a standard leg, an arm segment). This makes models easier to manage and adapt.
-   **Using Xacro for Complex Models**: For highly complex robots, manually writing URDF can become cumbersome. Xacro (XML Macros) is an XML macro language that allows for more concise and programmatic generation of URDF files, supporting parameters, conditionals, and file inclusions.
-   **Collision Detection and Avoidance Considerations**: Always define realistic collision geometries. Pay close attention to joint limits to prevent self-collisions. In dynamic simulations, ensure the collision properties accurately reflect the physical robot to avoid unexpected behaviors.
