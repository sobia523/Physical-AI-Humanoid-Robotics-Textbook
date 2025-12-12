# Chapter 5 - Micro-Project: Autonomous Navigation Pipeline

## Integrating Perception and Planning for Autonomous Humanoid Navigation

This micro-project serves as the culmination of Module 3, where you will integrate the concepts and tools learned in previous chapters. The goal is to create a complete autonomous navigation pipeline for a simulated humanoid robot, combining perception (VSLAM) and planning (Nav2) to enable it to navigate an environment and avoid obstacles.

### The Autonomous Navigation Pipeline

The full pipeline involves several interconnected components:
1.  **Isaac Sim**: The high-fidelity simulation environment hosting the humanoid robot and its sensors.
2.  **Synthetic Data Generation**: Sensors attached to the humanoid generate RGB-D and LiDAR data.
3.  **VSLAM (Isaac ROS)**: Processes sensor data to provide real-time localization (robot's pose) and build an environmental map.
4.  **Nav2**: Utilizes the VSLAM output (pose and map) to plan paths, manage local obstacle avoidance, and generate velocity commands for the robot.
5.  **Robot Control**: The robot's base controller receives velocity commands from Nav2 and translates them into physical movements in Isaac Sim.

#### **Visualization: Integrated Autonomous Navigation Pipeline**

```mermaid
graph TD
    A[Isaac Sim Environment (Humanoid + Sensors)] --> B{Synthetic Data Stream}
    B --> C{VSLAM (Isaac ROS)}
    C --> D{Localization & Mapping (Pose, Map)}
    D --> E{Nav2 Stack}
    E --> F{Robot Base Control Commands}
    F --> A
    E -- Waypoint Goals --> G[Mission Executor Script]
    D -- Map Data --> H[Lab Report Generator]
    F -- Trajectory Data --> H
    B -- Raw Sensor Data --> H
```
*Description*: A flowchart illustrating the integrated autonomous navigation pipeline. The Isaac Sim Environment with Humanoid and Sensors generates a Synthetic Data Stream. This stream feeds into VSLAM (Isaac ROS), which produces Localization & Mapping data (robot pose and map). This data then feeds into the Nav2 Stack, which generates Robot Base Control Commands back to the Isaac Sim Environment. Waypoint Goals are provided by a Mission Executor Script to the Nav2 Stack. The Lab Report Generator collects Map Data, Trajectory Data (from Robot Base Control), and Raw Sensor Data (from Synthetic Data Stream) to create reports.


## Testing Humanoid Robot Navigating Around Obstacles

To test the autonomous navigation pipeline, you will use the `full_autonomous_pipeline.launch.py` to bring up the integrated system and then command the robot using `autonomous_mission.py`. Obstacles can be dynamically added to the Isaac Sim environment to test avoidance capabilities.

### Launching the Full Autonomous Pipeline

The `full_autonomous_pipeline.launch.py` orchestrates the VSLAM and Nav2 components.

1.  **Examine `full_autonomous_pipeline.launch.py`**:
    Navigate to `module3-ai-robot-brain/ros2_packages/src/autonomous_pipeline/launch/full_autonomous_pipeline.launch.py`.
    ```python
    # module3-ai-robot-brain/ros2_packages/src/autonomous_pipeline/launch/full_autonomous_pipeline.launch.py
    import os
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource

    def generate_launch_description():
        isaac_ros_vslam_dir = get_package_share_directory('isaac_ros_vslam')
        nav2_humanoid_dir = get_package_share_directory('nav2_humanoid')
        
        return LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(isaac_ros_vslam_dir, 'launch', 'vslam_pipeline.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true' 
                }.items()
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_humanoid_dir, 'launch', 'nav2_humanoid.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true' 
                }.items()
            ),
        ])
    ```

### Executing an Autonomous Mission

The `autonomous_mission.py` script sends a series of waypoints to the integrated Nav2 stack.

1.  **Examine `autonomous_mission.py`**:
    Navigate to `module3-ai-robot-brain/ros2_packages/src/autonomous_pipeline/scripts/autonomous_mission.py`.
    ```python
    # module3-ai-robot-brain/ros2_packages/src/autonomous_pipeline/scripts/autonomous_mission.py
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateToPose
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    import time
    import random
    from tf_transformations import quaternion_from_euler # Needs to be installed: pip install transforms3d

    class AutonomousMissionExecutor(Node):
        def __init__(self):
            super().__init__('autonomous_mission_executor')
            self.get_logger().info('Autonomous Mission Executor Node started.')
            
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            )
            self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose', qos_profile=qos_profile)
            
            self.current_waypoint_index = 0
            self.waypoints = [] 

        def add_waypoint(self, x, y, yaw=0.0, frame_id='map'):
            waypoint = PoseStamped()
            waypoint.header.frame_id = frame_id
            waypoint.header.stamp = self.get_clock().now().to_msg()
            waypoint.pose.position.x = float(x)
            waypoint.pose.position.y = float(y)
            q = quaternion_from_euler(0.0, 0.0, float(yaw))
            waypoint.pose.orientation.x = q[0]
            waypoint.pose.orientation.y = q[1]
            waypoint.pose.orientation.z = q[2]
            waypoint.pose.orientation.w = q[3]
            self.waypoints.append(waypoint)
            self.get_logger().info(f"Waypoint added: ({x}, {y}, {yaw})")

        def start_mission(self):
            if not self.waypoints:
                self.get_logger().warn("No waypoints defined for the mission.")
                return

            self.get_logger().info("Starting autonomous navigation mission...")
            self._send_next_waypoint()

        def _send_next_waypoint(self):
            if self.current_waypoint_index < len(self.waypoints):
                waypoint_to_send = self.waypoints[self.current_waypoint_index]
                self.get_logger().info(f"Navigating to waypoint {self.current_waypoint_index + 1}/"
                                       f"{len(self.waypoints)}: x={waypoint_to_send.pose.position.x:.2f}, "
                                       f"y={waypoint_to_send.pose.position.y:.2f}")
                self.send_goal(waypoint_to_send)
            else:
                self.get_logger().info("All waypoints reached. Mission complete!")
                # rclpy.shutdown() 

        def send_goal(self, pose_stamped_msg):
            self.get_logger().info('Waiting for Nav2 action server...')
            if not self._action_client.wait_for_server(timeout_sec=10.0):
                self.get_logger().error("Nav2 action server not available!")
                return

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = pose_stamped_msg

            self._send_goal_future = self._action_client.send_goal_async(goal_msg)
            self._send_goal_future.add_done_callback(self.goal_response_callback)

        def goal_response_callback(self, future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn('Goal rejected by Nav2 server :(')
                self._send_next_waypoint() 
                return

            self.get_logger().info('Goal accepted :)')
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.get_result_callback)

        def get_result_callback(self, future):
            status = future.result().status
            result = future.result().result
            
            if status == 4: # GoalStatus.SUCCEEDED
                self.get_logger().info(f'Waypoint {self.current_waypoint_index + 1} reached successfully!')
                self.current_waypoint_index += 1
                self._send_next_waypoint()
            else:
                self.get_logger().error(f'Navigation to waypoint {self.current_waypoint_index + 1} failed with status: {status}')
                self.current_waypoint_index += 1 
                self._send_next_waypoint()

        def shutdown(self):
            self.get_logger().info("Shutting down Autonomous Mission Executor.")
            self._action_client.destroy()
            self.destroy_node()
            rclpy.shutdown()

    def main(args=None):
        rclpy.init(args=args)
        executor = AutonomousMissionExecutor()

        executor.add_waypoint(x=1.0, y=0.0, yaw=0.0)
        executor.add_waypoint(x=1.0, y=1.0, yaw=1.57) 
        executor.add_waypoint(x=0.0, y=1.0, yaw=3.14) 
        executor.add_waypoint(x=0.0, y=0.0, yaw=-1.57) 

        executor.start_mission()

        try:
            rclpy.spin(executor)
        except KeyboardInterrupt:
            executor.get_logger().info("Mission interrupted by user.")
        finally:
            executor.shutdown()

    if __name__ == '__main__':
        main()
    ```

### Steps to Run the Micro-Project

1.  **Launch Isaac Sim**: Start Isaac Sim and ensure it is ready.
2.  **Load Humanoid and Configure Sensors**:
    Execute the scripts from Chapter 2 to load your humanoid and attach sensors.
    ```bash
    python module3-ai-robot-brain/ros2_packages/src/isaac_sim_integration/scripts/load_humanoid.py
    python module3-ai-robot-brain/ros2_packages/src/isaac_sim_integration/scripts/configure_sensors.py
    ```
3.  **Build Your ROS 2 Workspace**:
    Ensure your `ros2_packages` workspace is built and sourced:
    ```bash
    cd Physical-AI-Humanoid-Robotics-Textbook/module3-ai-robot-brain/ros2_packages
    colcon build
    source install/setup.bash
    ```
4.  **Launch the Full Autonomous Pipeline**:
    Open a new terminal and launch the integrated pipeline:
    ```bash
    ros2 launch autonomous_pipeline full_autonomous_pipeline.launch.py
    ```
5.  **Start the Autonomous Mission**:
    In another terminal, execute the mission script to send waypoints. You can modify the `add_waypoint` calls in `autonomous_mission.py` to define your desired path.
    ```bash
    python module3-ai-robot-brain/ros2_packages/src/autonomous_pipeline/scripts/autonomous_mission.py
    ```
    Observe the humanoid robot in Isaac Sim navigating through the defined waypoints, dynamically avoiding any obstacles present in the environment.

## Including Lab Report with Maps, Sensor Data Plots, and Trajectory Visualization

After an autonomous mission, generating a lab report helps in analyzing the robot's performance.

### Generating Report Artifacts

The `generate_lab_report.py` script collects data and generates visualizations.

1.  **Examine `generate_lab_report.py`**:
    Navigate to `module3-ai-robot-brain/ros2_packages/src/autonomous_pipeline/scripts/generate_lab_report.py`.
    ```python
    # module3-ai-robot-brain/ros2_packages/src/autonomous_pipeline/scripts/generate_lab_report.py
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import OccupancyGrid, Path
    from sensor_msgs.msg import PointCloud2
    import matplotlib.pyplot as plt
    import numpy as np
    import os
    import time

    class LabReportGenerator(Node):
        def __init__(self):
            super().__init__('lab_report_generator')
            self.get_logger().info('Lab Report Generator Node started.')

            self.map_subscription = self.create_subscription(
                OccupancyGrid,
                '/map', 
                self.map_callback,
                10
            )
            self.path_subscription = self.create_subscription(
                Path,
                '/vslam/path', 
                self.path_callback,
                10
            )
            self.pointcloud_subscription = self.create_subscription(
                PointCloud2,
                '/vslam/map_points', 
                self.pointcloud_callback,
                10
            )

            self.occupancy_grid_data = None
            self.robot_path_data = None
            self.point_cloud_data = None
            self.output_dir = "lab_report_artifacts"
            os.makedirs(self.output_dir, exist_ok=True)

        def map_callback(self, msg):
            self.occupancy_grid_data = msg
            self.get_logger().info("Received Occupancy Grid Map.")

        def path_callback(self, msg):
            self.robot_path_data = msg
            self.get_logger().info("Received Robot Path.")

        def pointcloud_callback(self, msg):
            self.point_cloud_data = msg
            self.get_logger().info("Received Point Cloud Map.")

        def generate_report(self):
            self.get_logger().info("Generating lab report artifacts...")

            if self.occupancy_grid_data:
                self._plot_occupancy_grid(self.occupancy_grid_data)
            if self.robot_path_data:
                self._plot_trajectory(self.robot_path_data)
            if self.point_cloud_data:
                self.get_logger().info("Point cloud visualization requires advanced processing, skipping for placeholder.")

            self.get_logger().info(f"Lab report artifacts generated in {self.output_dir}")

        def _plot_occupancy_grid(self, occupancy_grid_msg):
            width = occupancy_grid_msg.info.width
            height = occupancy_grid_msg.info.height
            resolution = occupancy_grid_msg.info.resolution
            origin_x = occupancy_grid_msg.info.origin.position.x
            origin_y = occupancy_grid_msg.info.origin.position.y

            grid_data = np.array(occupancy_grid_msg.data).reshape((height, width))
            grid_data_flipped = np.flipud(grid_data) 

            plt.figure(figsize=(10, 10))
            plt.imshow(grid_data_flipped, cmap='gray', origin='lower',
                       extent=[origin_x, origin_x + width * resolution,
                               origin_y, origin_y + height * resolution])
            plt.title('Occupancy Grid Map')
            plt.xlabel('X position (m)')
            plt.ylabel('Y position (m)')
            plt.colorbar(label='Occupancy Probability (-1:unknown, 0:free, 100:occupied)')
            plt.savefig(os.path.join(self.output_dir, 'occupancy_grid_map.png'))
            plt.close()
            self.get_logger().info("Generated Occupancy Grid Map plot.")

        def _plot_trajectory(self, path_msg):
            x_coords = [pose_stamped.pose.position.x for pose_stamped in path_msg.poses]
            y_coords = [pose_stamped.pose.position.y for pose_stamped in path_msg.poses]

            plt.figure(figsize=(10, 10))
            plt.plot(x_coords, y_coords, marker='o', linestyle='-', color='blue')
            plt.title('Robot Trajectory')
            plt.xlabel('X position (m)')
            plt.ylabel('Y position (m)')
            plt.grid(True)
            plt.savefig(os.path.join(self.output_dir, 'robot_trajectory.png'))
            plt.close()
            self.get_logger().info("Generated Robot Trajectory plot.")

    def main(args=None):
        rclpy.init(args=args)
        generator = LabReportGenerator()

        generator.get_logger().info("Collecting data for 10 seconds...")
        end_time = time.time() + 10
        while rclpy.ok() and time.time() < end_time:
            rclpy.spin_once(generator, timeout_sec=0.1)
        
        generator.generate_report()
        generator.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```
2.  **Run the Script**:
    After successfully completing an autonomous mission, you can run this script to generate a lab report. Ensure the necessary ROS 2 topics (`/map`, `/vslam/path`, `/vslam/map_points`) were published during the mission.
    ```bash
    python module3-ai-robot-brain/ros2_packages/src/autonomous_pipeline/scripts/generate_lab_report.py
    ```
    This will generate image files (`.png`) in the `lab_report_artifacts` directory, visualizing the occupancy grid map and robot trajectory.

## Conclusion

This micro-project brought together all the components of the AI-Robot Brain module. You have successfully built an autonomous navigation pipeline for a humanoid robot in Isaac Sim, combining synthetic data generation, VSLAM for perception, and Nav2 for planning. The ability to generate and analyze lab report artifacts is crucial for evaluating and refining robotic systems.

## Glossary

*   **Autonomous Navigation Pipeline**: An integrated system that allows a robot to navigate an environment autonomously, typically involving perception, localization, mapping, and path planning.
*   **Mission Executor Script**: A script designed to sequence and send navigation goals (waypoints) to a robot's navigation stack.
*   **Lab Report Artifacts**: Data, plots, and visualizations generated from a robotic experiment or mission to document and analyze its performance.
*   **Occupancy Grid Map**: A grid-based representation of an environment where each cell stores the probability of being occupied by an obstacle.
*   **Trajectory Visualization**: A graphical representation of the path taken by a robot, often showing its position and orientation over time.

## References

*   [Nav2 Action Client Tutorial](https://navigation.ros.org/tutorials/docs/navigation2_action_client.html)
*   [Matplotlib Documentation](https://matplotlib.org/stable/contents/index.html)
*   [NumPy Documentation](https://numpy.org/doc/stable/)