from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the launch directory
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    params_file = LaunchConfiguration('params_file')
    
    # Define Nav2 parameters file (our custom one)
    our_params_file = PathJoinSubstitution([
        get_package_share_directory('nav2_humanoid_configs'),
        'params',
        'humanoid_nav2_params.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'autostart', default_value='true', description='Automatically startup the Nav2 stack'),
        DeclareLaunchArgument(
            'params_file',
            default_value=our_params_file,
            description='Full path to the Nav2 parameters file to use'),

        # Include the Nav2 bringup launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': '', # We'll build map with VSLAM, so no static map initially
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': params_file,
            }.items(),
        ),

        # Optional: Rviz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_nav2',
            arguments=['-d', PathJoinSubstitution([
                get_package_share_directory('nav2_humanoid_configs'),
                'rviz',
                'nav2_config.rviz' # Placeholder for Rviz config
            ])],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )
    ])
