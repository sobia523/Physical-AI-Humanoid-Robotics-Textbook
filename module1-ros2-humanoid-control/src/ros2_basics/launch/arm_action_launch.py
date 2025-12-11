from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='module1_ros2_humanoid_control',
            executable='arm_action_server.py',
            name='arm_action_server_node',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='module1_ros2_humanoid_control',
            executable='arm_action_client.py',
            name='arm_action_client_node',
            output='screen',
            emulate_tty=True,
            # Arguments can be passed like this to the client
            # arguments=['1.5'], # Example: target_position
        )
    ])
