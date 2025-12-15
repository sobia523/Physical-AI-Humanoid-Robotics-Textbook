import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_name = 'module2_ros2_packages'
    pkg_share_dir = get_package_share_directory(pkg_name)
    
    # Path to your URDF file with sensors
    urdf_file_path = os.path.join(
        pkg_share_dir,
        '..', # go up one level to module2-digital-twin
        'gazebo_simulations',
        'models',
        'simple_humanoid_sensors.urdf' # Use the URDF with sensors
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

    # Gazebo launch (using the existing world)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': os.path.join(
            pkg_share_dir,
            '..', # go up one level to module2-digital-twin
            'gazebo_simulations',
            'worlds',
            'basic_humanoid_world.world' # Use the world you created earlier
        )}.items()
    )

    # Spawn robot in Gazebo
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'simple_humanoid_sensors'],
        output='screen'
    )

    # Parameter Bridge for ros_gz_bridge (for LiDAR, IMU, Depth Camera)
    # The topic mapping must match the plugin configuration in your URDF
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/demo/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan', # LiDARS
            # Add IMU and Depth Camera topics here when you add them to the URDF
            # Example: '/imu_sensor@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            # '/camera@sensor_msgs/msg/Image[ignition.msgs.Image',
        ],
        output='screen'
    )

    # LiDAR Obstacle Detector node
    lidar_detector_node = Node(
        package=pkg_name,
        executable='lidar_obstacle_detector',
        name='lidar_obstacle_detector',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        gazebo_launch,
        spawn_entity_node,
        gz_bridge, # The bridge must be launched for sensor data to reach ROS 2
        lidar_detector_node,
    ])
