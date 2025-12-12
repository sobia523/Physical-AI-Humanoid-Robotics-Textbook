# Chapter 4 - Nav2 Path Planning for Bipedal Humanoids

## Introduction to Nav2 for Humanoid Robotics

Nav2 (Navigation2) is the standard ROS 2 navigation stack, providing a comprehensive set of tools for autonomous mobile robot navigation. While traditionally used for wheeled robots, its modular architecture allows adaptation for more complex platforms like bipedal humanoids. This chapter focuses on configuring Nav2 for a simulated humanoid, enabling it to plan safe trajectories and avoid obstacles.

### Key Components of Nav2

Nav2 comprises several interconnected nodes, including:
*   **Planner Server**: Generates a global plan (path) from start to goal.
*   **Controller Server**: Executes the global plan and generates local commands to the robot.
*   **Behavior Tree Navigator**: Orchestrates the planning and control, handling recovery behaviors.
*   **Costmaps**: Layers that represent the environment, including obstacles and inflated areas around them.
*   **AMCL/Localization**: Provides the robot's pose within the map (we will use VSLAM output).

#### **Visualization: Nav2 Stack Architecture for Humanoid Navigation**

```mermaid
graph TD
    A[VSLAM Output: Pose & Map] --> B{Nav2 Localization Node (e.g., AMCL or custom fusion)}
    B --> C{Global Costmap}
    C --> D{Planner Server}
    D --> E{Behavior Tree Navigator}
    E --> F{Controller Server}
    F --> G{Local Costmap}
    G --> H[Robot Base Controller]
    H --> I[Humanoid Robot (Isaac Sim)]
    I -- Sensor Data --> VSLAM Output
    E -- Waypoint Goals --> D
```
*Description*: A flowchart illustrating the Nav2 stack tailored for humanoid navigation. VSLAM Output (robot pose and map) feeds into a Nav2 Localization Node. This, along with a Global Costmap, informs the Planner Server. The Planner Server provides paths to the Behavior Tree Navigator, which orchestrates the mission by interacting with the Controller Server. The Controller Server, considering the Local Costmap, sends commands to the Robot Base Controller, which controls the Humanoid Robot in Isaac Sim. Sensor data from the Humanoid Robot feeds back into the VSLAM. Waypoint Goals are sent to the Planner Server.


## Setting Up the Navigation Stack for Simulated Robots

Configuring Nav2 involves defining various parameters, especially for costmaps and controller behaviors, to suit the robot's kinematics and sensor capabilities.

### Nav2 Parameter Configuration

The `nav2_params.yaml` file specifies the core parameters for the Nav2 stack.

1.  **Examine `nav2_params.yaml`**:
    Navigate to `module3-ai-robot-brain/ros2_packages/src/nav2_humanoid/config/nav2_params.yaml`.
    ```yaml
    # module3-ai-robot-brain/ros2_packages/src/nav2_humanoid/config/nav2_params.yaml
    # This file contains placeholder configurations for the Nav2 navigation stack adapted for a bipedal humanoid robot.
    # These parameters would typically be tuned for a specific robot and environment.
    # VSLAM output will be used for localization input.

    # Common Nav2 parameters
    planner_server:
      ros__parameters:
        use_sim_time: True
        planner_plugins: ["GridBased"]
        GridBased:
          plugin: "nav2_navfn_planner/NavfnPlanner"
          # Other planner specific parameters

    controller_server:
      ros__parameters:
        use_sim_time: True
        controller_plugins: ["FollowPath"]
        FollowPath:
          plugin: "nav2_controller/FollowPathController"
          # Other controller specific parameters (e.g., PID gains for bipedal movement)

    bt_navigator:
      ros__parameters:
        use_sim_time: True
        # Behavior tree file can be customized for humanoid specific behaviors
        # default_bt_filepath: "/path/to/humanoid_behavior_tree.xml" 

    # Localization (e.g., AMCL for map-based, or direct VSLAM output)
    # For this module, we assume VSLAM provides accurate pose.
    # If AMCL were used, it would subscribe to LiDAR scans and transform.
    amcl:
      ros__parameters:
        use_sim_time: True
        set_initial_pose: True
        # ... other AMCL parameters
        # For VSLAM integration, AMCL might be bypassed or configured to use VSLAM's pose directly
        # or a dedicated localization node that fuses VSLAM with IMU would publish to /amcl_pose.
        # We conceptualize that VSLAM directly provides a suitable pose estimate.

    # Map server (if using a static map, otherwise VSLAM will generate one)
    map_server:
      ros__parameters:
        use_sim_time: True
        # yaml_filename: "/path/to/static_map.yaml" # If using a pre-built map

    # Costmap configurations
    local_costmap:
      ros__parameters:
        use_sim_time: True
        global_frame: odom
        robot_base_frame: base_link
        update_frequency: 5.0
        publish_frequency: 2.0
        resolution: 0.05
        transform_tolerance: 0.5
        plugins: ["voxel_layer", "obstacle_layer", "inflation_layer"]
        voxel_layer:
          plugin: "nav2_voxel_grid/VoxelLayer"
          # ... parameters
        obstacle_layer:
          plugin: "nav2_costmap_2d/ObstacleLayer"
          # Subscribes to /scan topic (from simulated LiDAR)
          observation_sources: scan
          scan: {topic: /scan, max_obstacle_height: 2.0, clear_threshold: 0.25, mark_threshold: 0.75, data_type: LaserScan, clearing: True, marking: True, inf_is_valid: True}
        inflation_layer:
          plugin: "nav2_costmap_2d/InflationLayer"
          # ... parameters

    global_costmap:
      ros__parameters:
        use_sim_time: True
        global_frame: map
        robot_base_frame: base_link
        update_frequency: 1.0
        publish_frequency: 1.0
        resolution: 0.05
        transform_tolerance: 0.5
        plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
        static_layer:
          plugin: "nav2_costmap_2d/StaticLayer"
          # If VSLAM is generating a map, this layer could be dynamically updated or replaced.
          # For now, assumes a static map or a map provided by VSLAM.
        obstacle_layer:
          plugin: "nav2_costmap_2d/ObstacleLayer"
          observation_sources: scan
          scan: {topic: /scan, max_obstacle_height: 2.0, clear_threshold: 0.25, mark_threshold: 0.75, data_type: LaserScan, clearing: True, marking: True, inf_is_valid: True}
        inflation_layer:
          plugin: "nav2_costmap_2d/InflationLayer"

    # Life cycle manager (orchestrates Nav2 nodes)
    lifecycle_manager:
      ros__parameters:
        autostart: True
        node_names:
          - planner_server
          - controller_server
          - bt_navigator
          - amcl
          - map_server # Only if using static map or a specific map-generating node
          - local_costmap
          - global_costmap
    ```

### Nav2 Launch File

The `nav2_humanoid.launch.py` file brings up the entire Nav2 stack with the specified parameters.

1.  **Examine `nav2_humanoid.launch.py`**:
    Navigate to `module3-ai-robot-brain/ros2_packages/src/nav2_humanoid/launch/nav2_humanoid.launch.py`.
    ```python
    # module3-ai-robot-brain/ros2_packages/src/nav2_humanoid/launch/nav2_humanoid.launch.py
    import os
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch.substitutions import LaunchConfiguration, PythonExpression
    from launch_ros.actions import Node
    from nav2_common.launch import RewrittenYaml

    def generate_launch_description():
        nav2_humanoid_dir = get_package_share_directory('nav2_humanoid')
        nav2_bringup_dir = get_package_share_directory('nav2_bringup')

        nav2_params_path = os.path.join(nav2_humanoid_dir, 'config', 'nav2_params.yaml')

        namespace = LaunchConfiguration('namespace')
        use_sim_time = LaunchConfiguration('use_sim_time')
        autostart = LaunchConfiguration('autostart')
        params_file = LaunchConfiguration('params_file')
        default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
        map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')

        configured_params = RewrittenYaml(
            source_file=nav2_params_path,
            root_key=namespace,
            param_rewrites={},
            convert_input_to_system_param=True
        )

        return LaunchDescription([
            DeclareLaunchArgument(
                'namespace', default_value='',
                description='Top-level namespace'),
            DeclareLaunchArgument(
                'use_sim_time', default_value='true',
                description='Use simulation (Gazebo) clock if true'),
            DeclareLaunchArgument(
                'autostart', default_value='true',
                description='Automatically startup the nav2 stack'),
            DeclareLaunchArgument(
                'params_file',
                default_value=nav2_params_path,
                description='Full path to the Nav2 parameters file to use'),
            DeclareLaunchArgument(
                'default_bt_xml_filename',
                default_value=os.path.join(
                    nav2_bringup_dir,
                    'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
                description='Full path to the behavior tree xml file to use'),
            DeclareLaunchArgument(
                'map_subscribe_transient_local', default_value='false',
                description='Whether to set the map subscriber QoS to transient local'),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
                launch_arguments={
                    'namespace': namespace,
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'params_file': configured_params,
                    'default_bt_xml_filename': default_bt_xml_filename,
                    'map_subscribe_transient_local': map_subscribe_transient_local,
                }.items(),
            ),
        ])
    ```

## Hands-on: Execute a Waypoint Navigation Mission in a Simulated Environment

This section guides you through launching Nav2 and commanding a humanoid robot to a target waypoint.

### Commanding Waypoints

The `send_waypoints.py` script demonstrates how to send navigation goals to the Nav2 stack.

1.  **Examine `send_waypoints.py`**:
    Navigate to `module3-ai-robot-brain/ros2_packages/src/nav2_humanoid/scripts/send_waypoints.py`.
    ```python
    # module3-ai-robot-brain/ros2_packages/src/nav2_humanoid/scripts/send_waypoints.py
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateToPose
    import time

    class WaypointSender(Node):
        def __init__(self):
            super().__init__('waypoint_sender')
            self.get_logger().info('Waypoint Sender Node started.')
            self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        def send_goal(self, pose):
            self.get_logger().info('Waiting for action server...')
            self._action_client.wait_for_server()

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = pose

            self.get_logger().info(f'Sending goal: {goal_msg.pose.pose.position.x}, {goal_msg.pose.pose.position.y}')
            self._send_goal_future = self._action_client.send_goal_async(goal_msg)
            self._send_goal_future.add_done_callback(self.goal_response_callback)

        def goal_response_callback(self, future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().info('Goal rejected :(')
                return

            self.get_logger().info('Goal accepted :)')
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.get_result_callback)

        def get_result_callback(self, future):
            result = future.result().result
            self.get_logger().info('Result: {0}'.format(result.result))
            rclpy.shutdown()

    def main(args=None):
        rclpy.init(args=args)
        node = WaypointSender()

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map' 
        goal_pose.header.stamp = node.get_clock().now().to_msg()
        goal_pose.pose.position.x = 1.0
        goal_pose.pose.position.y = 0.0
        goal_pose.pose.orientation.w = 1.0 

        node.send_goal(goal_pose)

        rclpy.spin(node)

    if __name__ == '__main__':
        main()
    ```

### Steps to Run a Navigation Mission

1.  **Ensure Isaac Sim and VSLAM are Running**:
    Launch Isaac Sim, load your humanoid, configure sensors, and start the VSLAM pipeline as described in previous chapters. This ensures Nav2 has localization and mapping data.
    ```bash
    # Assuming previous scripts are running or launched via an orchestrator
    # python module3-ai-robot-brain/ros2_packages/src/isaac_sim_integration/scripts/load_humanoid.py
    # python module3-ai-robot-brain/ros2_packages/src/isaac_sim_integration/scripts/configure_sensors.py
    # ros2 launch isaac_ros_vslam vslam_pipeline.launch.py
    ```
2.  **Build Your ROS 2 Workspace**:
    Ensure your `ros2_packages` workspace is built:
    ```bash
    cd Physical-AI-Humanoid-Robotics-Textbook/module3-ai-robot-brain/ros2_packages
    colcon build
    source install/setup.bash
    ```
3.  **Launch Nav2 for Humanoid**:
    Open a new terminal and launch the Nav2 stack:
    ```bash
    ros2 launch nav2_humanoid nav2_humanoid.launch.py
    ```
    You should see the Nav2 nodes start up.
4.  **Send a Waypoint Goal**:
    In another terminal, send a navigation goal using the `send_waypoints.py` script. Adjust the `goal_pose.pose.position.x` and `y` values in the script to your desired target.
    ```bash
    python module3-ai-robot-brain/ros2_packages/src/nav2_humanoid/scripts/send_waypoints.py
    ```
    Observe the humanoid robot in Isaac Sim attempting to navigate to the specified waypoint, avoiding any obstacles present in the environment. You can also monitor the planned path and robot's movement in Rviz.

## Conclusion

This chapter demonstrated how to adapt and configure the powerful Nav2 framework for bipedal humanoid navigation in a simulated environment. You've learned to integrate localization data from VSLAM, define navigation parameters, and command the robot to reach target waypoints while performing obstacle avoidance.

## Glossary

*   **Nav2 (Navigation2)**: The standard ROS 2 navigation stack for autonomous mobile robot navigation.
*   **Planner Server**: A Nav2 component responsible for generating a global path from the robot's current position to a goal.
*   **Controller Server**: A Nav2 component responsible for executing the global path and generating velocity commands for the robot.
*   **Behavior Tree Navigator**: A Nav2 component that uses behavior trees to orchestrate the robot's navigation tasks, including recovery behaviors.
*   **Costmap**: A 2D or 3D grid representing the traversability of an environment, used by Nav2 for path planning and obstacle avoidance.
*   **Waypoint**: A target location or series of locations (position and orientation) that a robot is commanded to reach.
*   **AMCL (Adaptive Monte Carlo Localization)**: A popular probabilistic localization algorithm often used in Nav2, though in this module, VSLAM provides the localization.
*   **Localization**: The process of determining a robot's precise position and orientation within a given map.

## References

*   [Nav2 Documentation](https://navigation.ros.org/)
*   [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/index.html)
*   [ROS 2 Launch System](https://docs.ros.org/en/humble/Tutorials/Launch-Files/Creating-a-Launch-File.html)