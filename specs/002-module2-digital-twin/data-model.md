# Data Model: Module 2 — The Digital Twin (Gazebo & Unity)

## Entities:

### 1. Digital Twin
*   **Description**: A virtual model of a physical robot and its environment.
*   **Attributes**: (Implicitly includes representations in Gazebo World and Unity Scene)
*   **Relationships**: Comprises a Humanoid Robot, Gazebo World, Unity Scene, and Sensors. Interacts with ROS 2 Nodes.

### 2. Humanoid Robot
*   **Description**: The specific type of robot being simulated.
*   **Attributes**: (Implicitly defined by URDF, dynamics, joint states)
*   **Relationships**: Part of a Digital Twin. Defined by URDF.

### 3. Gazebo World
*   **Description**: The simulated environment created in Gazebo.
*   **Attributes**: (Implicitly includes physics properties like gravity, collision models)
*   **Relationships**: Hosts Humanoid Robot and Sensors. Part of a Digital Twin.

### 4. Unity Scene
*   **Description**: The high-fidelity rendering environment created in Unity.
*   **Attributes**: (Implicitly includes lighting, textures, interaction logic)
*   **Relationships**: Provides visualization for Humanoid Robot and Sensors. Part of a Digital Twin.

### 5. Sensors
*   **Description**: Simulated components like LiDAR, IMUs, Depth Cameras, providing data.
*   **Attributes**: (Implicitly includes sensor type, data output format, noise models)
*   **Relationships**: Located within Gazebo World and visualized in Unity Scene. Provides data to ROS 2 Nodes.

### 6. ROS 2 Nodes
*   **Description**: Software processes interacting with simulated sensor data.
*   **Attributes**: (Implicitly includes topic subscriptions/publications, service calls)
*   **Relationships**: Interacts with Sensors. Integrates simulation data into robot control/perception pipelines.

### 7. URDF (Unified Robot Description Format)
*   **Description**: Used to define robot models (kinematics, visuals, collision properties).
*   **Attributes**: (Implicitly includes links, joints, materials)
*   **Relationships**: Defines the structure and properties of a Humanoid Robot.
