import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get package directories
    isaac_ros_vslam_dir = get_package_share_directory('isaac_ros_vslam')
    nav2_humanoid_dir = get_package_share_directory('nav2_humanoid')
    
    return LaunchDescription([
        # Launch the VSLAM pipeline (from Chapter 3)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(isaac_ros_vslam_dir, 'launch', 'vslam_pipeline.launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true' # Assuming simulation time is used
            }.items()
        ),

        # Launch the Nav2 stack for the humanoid (from Chapter 4)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_humanoid_dir, 'launch', 'nav2_humanoid.launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true' # Assuming simulation time is used
            }.items()
        ),

        # Additional nodes for the micro-project can be added here
        # E.g., a node to control the humanoid's base velocity based on Nav2 commands,
        # or a node to simulate obstacles, etc.
    ])
