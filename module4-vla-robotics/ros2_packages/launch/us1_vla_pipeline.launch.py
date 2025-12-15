from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vla_robotics_package',
            executable='voice_capture_node.py',
            name='voice_capture_node',
            output='screen'
        ),
        Node(
            package='vla_robotics_package',
            executable='voice_transcription_node.py',
            name='voice_transcription_node',
            output='screen'
        ),
        Node(
            package='vla_robotics_package',
            executable='cognitive_planner_node.py',
            name='cognitive_planner_node',
            output='screen'
        ),
    ])
