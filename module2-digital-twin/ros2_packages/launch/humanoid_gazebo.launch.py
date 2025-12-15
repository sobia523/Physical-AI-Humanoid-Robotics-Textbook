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