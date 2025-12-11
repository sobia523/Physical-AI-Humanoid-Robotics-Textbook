import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Get the launch directory
    pkg_share_dir = get_package_share_directory('module1_ros2_humanoid_control')
    urdf_file_path = os.path.join(pkg_share_dir, 'urdf_models', 'simple_humanoid_arm.urdf')

    # For now, just load the URDF directly
    with open(urdf_file_path, 'r') as infp:
        robot_description_content = infp.read()
    robot_description = {'robot_description': robot_description_content}

    return LaunchDescription([
        # Publish the robot state
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description]),

        # Visualize in Rviz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share_dir, 'rviz', 'urdf_config.rviz')], # Placeholder rviz config
        ),
        # A joint state publisher GUI to manually control joints for visualization
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        )
    ])
