# Data Model for Module 3: The AI-Robot Brain (NVIDIA Isaac™)

This document outlines the key data entities and their relationships for Module 3.

## Humanoid Robot (Simulated)

*   **Attributes**: Joint names, link properties, sensor configurations (LiDAR, RGB-D camera, IMU), articulation body properties (for Isaac Sim).
*   **Relationships**: Part of Isaac Sim environment, interacts with Isaac ROS nodes.
*   **Validation**: USD schema/URDF (if imported) validation.

## Isaac Sim Environment

*   **Attributes**: Scene geometry, static/dynamic obstacles, lighting, physics settings.
*   **Relationships**: Contains robot models, provides sensor data.

## Synthetic Sensor Data

*   **Attributes**: Timestamp, frame_id, sensor-specific data (e.g., `sensor_msgs/Image`, `sensor_msgs/PointCloud2`), ground truth data.
*   **Validation**: Conforms to ROS 2 message types, consistency with ground truth.

## VSLAM Output

*   **Attributes**: Estimated robot pose (`geometry_msgs/PoseStamped`), map (`nav_msgs/OccupancyGrid`), point cloud map (`sensor_msgs/PointCloud2`).
*   **Validation**: Consistency between pose and map, drift characteristics.

## Nav2 Configuration

*   **Attributes**: Global/local costmaps, planners (e.g., DWB, TEB), controllers, recovery behaviors, robot footprint.
*   **Validation**: Parameters tuned for bipedal humanoid motion.
