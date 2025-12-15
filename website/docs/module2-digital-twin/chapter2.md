# Chapter 2: Gazebo Physics Simulation

This chapter dives into the practical aspects of simulating humanoid robots using Gazebo. We will cover setting up a Gazebo world, simulating fundamental physics principles like gravity and collisions, and defining robot dynamics using URDF. A mini-lab will guide you through loading a humanoid robot and simulating simple movements.

## 2.1 Setting Up a Gazebo World

A Gazebo world (`.world` file) defines the environment for your simulation. It includes static objects, lights, sensors, and the physics engine configuration.

### Basic World Structure

A `.world` file is an XML document that specifies the entire simulation environment. Here's a minimal example:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <physics type="ode">
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
    </physics>
    <!-- Add your robot model here -->
  </world>
</sdf>
```

-   `<sdf version="1.6">`: Specifies the Simulation Description Format (SDF) version.
-   `<world name="default">`: Defines the world.
-   `<include>`: Used to import existing models like `sun` (for lighting) and `ground_plane`.
-   `<physics type="ode">`: Configures the physics engine. `ode` (Open Dynamics Engine) is a common choice.

### Key Physics Parameters

-   `<max_step_size>`: The maximum time step size the physics engine will use. Smaller values increase accuracy but decrease simulation speed.
-   `<real_time_factor>`: The ratio of simulated time to real time. A value of 1.0 means the simulation attempts to run at real-time speed.
-   `<real_time_update_rate>`: The frequency at which Gazebo updates its physics. A higher rate means more frequent physics calculations.

## 2.2 Simulating Gravity, Collisions, and Robot Dynamics

### Gravity

Gravity is a fundamental force in any realistic simulation. In Gazebo, it's typically defined at the world level. By default, `model://ground_plane` includes gravity. You can explicitly set it within the `<gravity>` tag under `<physics>`.

Example:
```xml
    <physics type="ode">
      <gravity>0 0 -9.8</gravity> <!-- Standard gravity in negative Z direction -->
      <!-- ... other physics parameters ... -->
    </physics>
```

---
**Figure 2.1: Gravity Vector Diagram**

*This diagram shows a simple coordinate system (X, Y, Z axes). A downward arrow along the negative Z-axis, originating from the center of the world, represents the default gravity vector of [0 0 -9.8]. A humanoid robot model stands on a ground plane, illustrating the effect of gravity pulling it downwards.*
---

### Collisions

Collision detection is crucial for robots to interact realistically with their environment and themselves. In Gazebo, collision shapes are defined within the `<collision>` tag of a robot's link in its URDF/SDF model.

-   **Collision Geometry**: Can be primitive shapes (box, sphere, cylinder) or meshes. Using primitives is generally more computationally efficient.
-   **Collision vs. Visual**: A link can have separate collision and visual geometries. This is common to simplify collision calculations while maintaining detailed visual appearance.

---
**Figure 2.2: Collision vs. Visual Geometry Diagram**

*This diagram presents two representations of a robot link side-by-side. On the left, the "Visual Geometry" shows a detailed, smoothly rendered 3D model of a robot part (e.g., a hand). On the right, the "Collision Geometry" for the same robot part is represented by a simpler primitive shape (e.g., a box or a sphere) that encapsulates the visual geometry. Overlapping areas are highlighted to show how a simplified collision model can approximate a complex visual model for efficient physics calculations.*
---

### Robot Dynamics (URDF)

The Unified Robot Description Format (URDF) is an XML format for describing a robot. While primarily used by ROS, Gazebo can also interpret URDF files. A URDF defines:

-   **Links**: The rigid bodies of the robot. Each link has a mass, inertia, visual, and collision properties.
    ```xml
    <link name="base_link">
      <visual>
        <geometry>
          <box size="0.1 0.1 0.1"/>
        </geometry>
      </visual>
      <collision>
        <geometry>
          <box size="0.1 0.1 0.1"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="1.0"/>
        <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
      </inertial>
    </link>
    ```
-   **Joints**: Connect links and define their relative motion (e.g., revolute, prismatic, fixed).
    ```xml
    <joint name="revolute_joint" type="revolute">
      <parent link="base_link"/>
      <child link="upper_arm_link"/>
      <origin xyz="0 0 0.05"/>
      <axis xyz="0 1 0"/>
      <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
    </joint>
    ```
-   **Transmissions**: Describe the relationship between actuators and joints, important for controlling the robot.

---
**Figure 2.3: URDF Link and Joint Diagram**

*This diagram shows two rectangular blocks labeled "Link A" (parent) and "Link B" (child), connected by a pivot point representing a "Joint". Arrows indicate the axis of rotation for the joint. Each link is accompanied by callouts pointing to its visual, collision, and inertial properties. The joint has callouts for its type (e.g., revolute), parent/child links, origin, axis, and limits.*
---

## 2.3 Mini-Lab: Load a Humanoid URDF and Simulate Simple Movement

In this mini-lab, you will:
1.  Create a basic Gazebo world.
2.  Define a simple humanoid robot using URDF.
3.  Launch the robot in Gazebo.
4.  Control its joints using a ROS 2 Python node.

### Step 1: Create a Basic Gazebo World (`basic_humanoid_world.world`)

Navigate to `module2-digital-twin/gazebo_simulations/worlds/` and create `basic_humanoid_world.world`.

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="humanoid_world">
    <include>
      <uri>model://sun</uri>
    </include>
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
  </world>
</sdf>
```

### Step 2: Create a Simple Humanoid URDF Model (`simple_humanoid.urdf`)

Navigate to `module2-digital-twin/gazebo_simulations/models/` and create `simple_humanoid.urdf`. This example defines a simple two-link arm, which is a common building block for humanoid robots.

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

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

  <!-- Gazebo specific settings -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>
  <gazebo reference="upper_arm_link">
    <material>Gazebo/Red</material>
  </gazebo>
  <gazebo>
    <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
      <parameters>$(find module2_ros2_packages)/config/simple_humanoid_controller.yaml</parameters>
    </plugin>
  </gazebo>
</robot>
```
**Note**: The URDF references a `simple_humanoid_controller.yaml` which we will create shortly.

### Step 3: Create ROS 2 Python Node for Joint Control (`simple_joint_controller.py`)

Navigate to `module2-digital-twin/ros2_packages/src/gazebo_humanoid_control/` and create the directory `gazebo_humanoid_control`. Inside, create `simple_joint_controller.py`.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Duration

class SimpleJointController(Node):

    def __init__(self):
        super().__init__('simple_joint_controller')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/shoulder_controller/commands', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0.0
        self.direction = 1

    def timer_callback(self):
        msg = Float64MultiArray()
        # Oscillate between -1.57 and 1.57 radians (approx -90 to 90 degrees)
        self.i += self.direction * 0.1
        if self.i > 1.57:
            self.i = 1.57
            self.direction = -1
        elif self.i < -1.57:
            self.i = -1.57
            self.direction = 1
        
        msg.data = [self.i] # Assuming one joint for now
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{self.i}"')

def main(args=None):
    rclpy.init(args=args)
    simple_joint_controller = SimpleJointController()
    rclpy.spin(simple_joint_controller)
    simple_joint_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
**Important**: Update `module2-digital-twin/ros2_packages/setup.py` to include this executable.
Add the following under `entry_points`:
```python
        'console_scripts': [
            'simple_joint_controller = module2_ros2_packages.gazebo_humanoid_control.simple_joint_controller:main',
        ],
```

### Step 4: Create ROS 2 Launch File (`humanoid_gazebo.launch.py`)

Navigate to `module2-digital-twin/ros2_packages/launch/` and create `humanoid_gazebo.launch.py`.

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_name = 'module2_ros2_packages'
    pkg_share_dir = get_package_share_directory(pkg_name)
    
    # Path to your URDF file
    urdf_file_path = os.path.join(
        pkg_share_dir,
        '..', # go up one level to module2-digital-twin
        'gazebo_simulations',
        'models',
        'simple_humanoid.urdf'
    )

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_file_path).read(),
                     'use_sim_time': True}]
    )

    # Joint State Publisher GUI node (optional, for manual control)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': os.path.join(
            get_package_share_directory(pkg_name),
            '..', # go up one level to module2-digital-twin
            'gazebo_simulations',
            'worlds',
            'basic_humanoid_world.world'
        )}.items()
    )

    # Spawn robot in Gazebo
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'simple_humanoid'],
        output='screen'
    )

    # Simple Joint Controller node
    simple_joint_controller_node = Node(
        package=pkg_name,
        executable='simple_joint_controller',
        name='simple_joint_controller',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        # joint_state_publisher_gui_node, # Uncomment for manual GUI control
        gazebo_launch,
        spawn_entity_node,
        simple_joint_controller_node,
    ])
```

### Step 5: Create Controller Configuration (`simple_humanoid_controller.yaml`)

Navigate to `module2-digital-twin/ros2_packages/config/` and create the directory `config`. Inside, create `simple_humanoid_controller.yaml`.

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    shoulder_controller:
      type: joint_state_broadcaster/JointStateBroadcaster

    shoulder_controller:
      type: velocity_controllers/JointVelocityController
      joints:
        - shoulder_joint
```
**Note**: The controller type should align with the control interface you want to use. `JointVelocityController` is used here for simplicity. For position control, you might use `JointPositionController`.

### Step 6: Build and Run

1.  Navigate to the project root: `cd Physical-AI-Humanoid-Robotics-Textbook`
2.  Build your ROS 2 package:
    ```bash
    colcon build --packages-select module2_ros2_packages
    ```
3.  Source your workspace:
    ```bash
    source install/setup.bash
    ```
4.  Launch the simulation and controller:
    ```bash
    ros2 launch module2_ros2_packages humanoid_gazebo.launch.py
    ```

You should see Gazebo launch with your simple humanoid arm, and the arm should start oscillating as controlled by the `simple_joint_controller` node.