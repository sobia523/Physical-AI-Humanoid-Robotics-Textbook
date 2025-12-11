from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='module1_ros2_humanoid_control',
            executable='gripper_service_server.py',
            name='gripper_service_server_node',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='module1_ros2_humanoid_control',
            executable='gripper_service_client.py',
            name='gripper_service_client_node',
            output='screen',
            emulate_tty=True,
            # Arguments can be passed like this to the client
            # arguments=['true'], # Example: to open gripper
        )
    ])
