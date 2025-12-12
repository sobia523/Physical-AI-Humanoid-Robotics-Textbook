# Conceptual API Contracts: AI-Robot Brain (NVIDIA Isaac)

**Branch**: `001-isaac-ai-robot-brain` | **Date**: 2025-12-12 | **Spec**: [specs/001-isaac-ai-robot-brain/spec.md](specs/001-isaac-ai-robot-brain/spec.md)
**Purpose**: To outline the primary ROS 2 interfaces (topics, services, actions) that will be used for communication between different components within the AI-Robot Brain module. These are conceptual contracts, guiding the implementation of ROS 2 nodes and their interactions.

## ROS 2 Communication Interfaces

### 1. VSLAM Pipeline Interfaces (Isaac ROS VSLAM Node)

*   **Input Topics**:
    *   `/isaac_sim/camera/rgb`: `sensor_msgs/msg/Image` (RGB image data from simulated camera)
    *   `/isaac_sim/camera/depth`: `sensor_msgs/msg/Image` (Depth image data from simulated camera)
    *   `/isaac_sim/imu`: `sensor_msgs/msg/Imu` (IMU data from simulated sensor)
    *   *(Note: Specific topic names may vary based on Isaac Sim and Isaac ROS configuration, but the data types remain consistent.)*
*   **Output Topics**:
    *   `/vslam/pose`: `geometry_msgs/msg/PoseStamped` (Estimated 3D pose of the robot)
    *   `/vslam/map_points`: `sensor_msgs/msg/PointCloud2` (Point cloud representation of the map)
    *   `/vslam/path`: `nav_msgs/msg/Path` (History of the robot's estimated trajectory)
    *   *(Note: Output topics provide localization and mapping data for Nav2 and visualization.)*

### 2. Navigation Stack (Nav2) Interfaces

*   **Input Topics**:
    *   `/amcl_pose` or `/vslam/pose`: `geometry_msgs/msg/PoseWithCovarianceStamped` or `geometry_msgs/msg/PoseStamped` (Robot's localization estimate, ideally from VSLAM)
    *   `/map`: `nav_msgs/msg/OccupancyGrid` (2D occupancy grid map of the environment)
    *   `/scan`: `sensor_msgs/msg/LaserScan` (LiDAR scan data for local obstacle avoidance)
*   **Output Actions**:
    *   `/navigate_to_pose`: `nav2_msgs/action/NavigateToPose` (Action for requesting the robot to navigate to a specific pose)
        *   **Goal**: `geometry_msgs/msg/PoseStamped` (Target pose)
        *   **Result**: `nav2_msgs/action/NavigateToPose/Result` (Success/failure, final pose)
        *   **Feedback**: `nav2_msgs/action/NavigateToPose/Feedback` (Current pose, distance remaining)
*   **Output Topics**:
    *   `/cmd_vel`: `geometry_msgs/msg/Twist` (Velocity commands sent to the robot's base controller)

### 3. Robot Control Interface (Humanoid Base Controller)

*   **Input Topics**:
    *   `/cmd_vel`: `geometry_msgs/msg/Twist` (Receives velocity commands from Nav2)
*   **Internal Interfaces (Example)**:
    *   Topics/Services for controlling individual `Humanoid Robot` `joints` (e.g., `std_msgs/msg/Float64` for position commands, or custom `control_msgs` if applicable, as defined in Module 1).
    *(Note: The specifics of direct joint control will build upon concepts from Module 1's ROS 2 basics and URDF models.)*

### 4. Synthetic Data Generation Interface (Isaac Sim Script)

*   **Mechanism**: Python scripts executed within Isaac Sim to configure sensors and trigger data capture.
*   **Output**: Files containing `Synthetic Sensor Data` (e.g., `.png` for RGB-D, `.npy` for LiDAR point clouds, or streamed directly to ROS 2 topics).
*   **Conceptual "API"**: Python function calls within the Isaac Sim scripting environment for sensor manipulation and data saving/streaming.

## Data Types Overview

The module will primarily utilize standard ROS 2 message types (`sensor_msgs`, `geometry_msgs`, `nav_msgs`, `std_msgs`) for communication, reinforcing concepts from Module 1. Custom message types will be avoided unless absolutely necessary for clarity or specific Isaac ROS functionalities.
