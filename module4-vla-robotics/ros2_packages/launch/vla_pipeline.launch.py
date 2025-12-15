from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_name = 'vla_robotics_package'
    pkg_share_dir = get_package_share_directory(pkg_name)
    
    gazebo_world_path = os.path.join(pkg_share_dir, 'simulations', 'gazebo', 'vla_world.world')

    return LaunchDescription([
        # Launch argument for simulation type (Gazebo/Unity) - not fully implemented yet
        DeclareLaunchArgument(
            'simulation_type',
            default_value='gazebo',
            description='Choose simulation environment (gazebo or unity)'
        ),

        # Launch Gazebo (if simulation_type is gazebo)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    get_package_share_directory('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                ])
            ]),
            launch_arguments={'world': gazebo_world_path}.items(),
            condition=LaunchConfiguration('simulation_type') == 'gazebo'
        ),

        Node(
            package=pkg_name,
            executable='voice_capture_node.py',
            name='voice_capture_node',
            output='screen'
        ),
        Node(
            package=pkg_name,
            executable='voice_transcription_node.py',
            name='voice_transcription_node',
            output='screen'
        ),
        Node(
            package=pkg_name,
            executable='cognitive_planner_node.py',
            name='cognitive_planner_node',
            output='screen'
        ),
        Node(
            package=pkg_name,
            executable='perception_node.py',
            name='perception_node',
            output='screen'
        ),
        Node(
            package=pkg_name,
            executable='task_executor_node.py',
            name='task_executor_node',
            output='screen'
        ),
        Node(
            package=pkg_name,
            executable='manipulation_controller_node.py',
            name='manipulation_controller_node',
            output='screen'
        ),
        # Add a placeholder for a robot state publisher and joint state publisher if a URDF is available
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen',
        #     parameters=[{'robot_description': '<robot_description_content>'}]
        # ),
    ])
