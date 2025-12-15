# API Contracts for Module 3: The AI-Robot Brain (NVIDIA Isaac™)

This document outlines the API contracts, primarily ROS 2 topics and message types, for integrating different components within Module 3.

## ROS 2 Topics for Isaac Sim Sensor Data

*   **LiDAR**: `/lidar_scan` (`sensor_msgs/LaserScan` or `sensor_msgs/PointCloud2`)
*   **RGB-D Camera**: `/camera/rgb/image_raw`, `/camera/depth/image_raw`, `/camera/points` (`sensor_msgs/Image`, `sensor_msgs/PointCloud2`)
*   **IMU**: `/imu/data` (`sensor_msgs/Imu`)

## ROS 2 Topics for VSLAM Input/Output

*   **Input**: `/camera/rgb/image_raw`, `/camera/depth/image_raw`, `/imu/data`
*   **Output**: `/vslam/pose`, `/vslam/map` (`geometry_msgs/PoseStamped`, `nav_msgs/OccupancyGrid`)

## ROS 2 Topics for Nav2 Input/Output

*   **Input**: `/map`, `/tf`, `/scan` (or other sensor data), `/goal_pose` (`nav_msgs/OccupancyGrid`, `tf2_msgs/TFMessage`, `sensor_msgs/LaserScan`, `geometry_msgs/PoseStamped`)
*   **Output**: `/cmd_vel` (`geometry_msgs/Twist`), `/plan`, `/local_plan`

## Isaac Sim-ROS 2 Bridge Configuration

*   Specific topic remapping and data type conversions.
