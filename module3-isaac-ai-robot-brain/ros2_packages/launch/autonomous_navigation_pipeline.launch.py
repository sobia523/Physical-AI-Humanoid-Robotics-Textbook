from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    isaac_ros_vslam_configs_dir = get_package_share_directory('isaac_ros_vslam_configs')
    nav2_humanoid_configs_dir = get_package_share_directory('nav2_humanoid_configs')

    # Include VSLAM Launch
    vslam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(isaac_ros_vslam_configs_dir, 'launch', 'vslam_launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items() # Assuming simulation time
    )

    # Include Nav2 Launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_humanoid_configs_dir, 'launch', 'humanoid_nav2_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true', # Assuming simulation time
            'params_file': os.path.join(nav2_humanoid_configs_dir, 'params', 'humanoid_nav2_params.yaml')
        }.items()
    )

    return LaunchDescription([
        vslam_launch,
        nav2_launch
    ])
