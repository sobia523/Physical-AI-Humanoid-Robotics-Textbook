import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'module2_digital_twin'
    # Get the directory of the current launch file
    current_launch_dir = os.path.dirname(__file__)

    # Construct paths relative to the launch file's directory
    urdf_file = os.path.join(current_launch_dir, '..', '..', 'gazebo_simulations', 'models', 'humanoid.urdf')
    world_file = os.path.join(current_launch_dir, '..', '..', 'gazebo_simulations', 'worlds', 'small_room.world')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Start Gazebo server
    start_gazebo_server_cmd = Node(
        package='gazebo_ros',
        executable='gazebo_ros_factory',
        output='screen',
        arguments=['-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', '-g', world_file],
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': open(urdf_file, 'r').read()}],
    )

    # Simple Navigator Node
    simple_navigator_node = Node(
        package='module2_digital_twin', # The package name defined in setup.py
        executable='simple_navigator', # The entry point defined in setup.py
        name='simple_navigator',
        output='screen',
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        start_gazebo_server_cmd,
        robot_state_publisher_node,
        simple_navigator_node,
    ])

