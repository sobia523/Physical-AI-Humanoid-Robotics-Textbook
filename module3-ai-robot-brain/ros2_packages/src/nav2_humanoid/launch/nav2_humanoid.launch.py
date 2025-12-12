import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Get the package directory
    nav2_humanoid_dir = get_package_share_directory('nav2_humanoid')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Path to the Nav2 parameters file
    nav2_params_path = os.path.join(nav2_humanoid_dir, 'config', 'nav2_params.yaml')

    # Arguments
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')

    # Create a RewrittenYaml action to handle parameter substitutions
    configured_params = RewrittenYaml(
        source_file=nav2_params_path,
        root_key=namespace,
        param_rewrites={},
        convert_input_to_system_param=True
    )

    return LaunchDescription([
        # Declare launch arguments
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

        # Include the main Nav2 launch file
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

        # You might also want to launch a robot_state_publisher here to publish TF frames
        # and potentially a static_tf_publisher for sensor transforms if not part of URDF
        # (This is a conceptual placeholder)
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description_content}],
        # ),
    ])
