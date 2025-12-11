from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='module1_ros2_humanoid_control',
            executable='mock_joint_controller.py',
            name='mock_joint_controller_node',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='module1_ros2_humanoid_control',
            executable='simple_agent.py',
            name='simple_agent_node',
            output='screen',
            emulate_tty=True,
        )
    ])
