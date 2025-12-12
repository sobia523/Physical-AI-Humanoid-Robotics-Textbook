import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the share directory for your package (e.g., isaac_sim_integration)
    # This needs to be correctly set up in package.xml and CMakeLists.txt for a real ROS 2 package
    # For now, we'll assume scripts are directly in src/isaac_sim_integration/scripts
    isaac_sim_integration_share_dir = get_package_share_directory('isaac_sim_integration')

    # Paths to your Python scripts
    load_humanoid_script = os.path.join(
        # In a real scenario, this would be in install/isaac_sim_integration/lib/isaac_sim_integration/
        # For this placeholder, we reference directly from src
        get_package_share_directory('isaac_sim_integration'),
        '../scripts/load_humanoid.py'
    )
    configure_sensors_script = os.path.join(
        get_package_share_directory('isaac_sim_integration'),
        '../scripts/configure_sensors.py'
    )
    capture_rgbd_script = os.path.join(
        get_package_share_directory('isaac_sim_integration'),
        '../scripts/capture_rgbd.py'
    )
    capture_lidar_script = os.path.join(
        get_package_share_directory('isaac_sim_integration'),
        '../scripts/capture_lidar.py'
    )

    return LaunchDescription([
        # Example: Launching the script to load a humanoid and environment
        # Note: Isaac Sim scripts are typically run directly within Isaac Sim's Python environment
        # or via a wrapper that launches Isaac Sim and then executes the script.
        # This ROS 2 launch file would typically trigger a ROS 2 node that interfaces with Isaac Sim.
        # For simplicity and to represent the task, we are showing how a script *could* be launched.
        # Actual integration requires more complex setup (e.g., using Isaac ROS Omniverse nodes or dedicated bridges).

        Node(
            package='isaac_sim_integration',  # Placeholder package name
            executable='load_humanoid.py',   # Assumes this script is made executable and part of package
            name='load_humanoid_node',
            output='screen',
            # arguments=['--ros-args', '--log-level', 'info'] # Example ROS 2 arguments
        ),
        Node(
            package='isaac_sim_integration',
            executable='configure_sensors.py',
            name='configure_sensors_node',
            output='screen',
            # arguments=['--ros-args', '--log-level', 'info']
        ),
        Node(
            package='isaac_sim_integration',
            executable='capture_rgbd.py',
            name='capture_rgbd_node',
            output='screen',
        ),
        Node(
            package='isaac_sim_integration',
            executable='capture_lidar.py',
            name='capture_lidar_node',
            output='screen',
        ),
        # More nodes for specific functionalities can be added here
    ])
