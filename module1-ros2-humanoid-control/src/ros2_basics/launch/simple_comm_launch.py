from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='module1_ros2_humanoid_control', # ROS 2 package name from package.xml
            executable='simple_publisher.py',
            name='simple_publisher_node',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='module1_ros2_humanoid_control', # ROS 2 package name from package.xml
            executable='simple_subscriber.py',
            name='simple_subscriber_node',
            output='screen',
            emulate_tty=True,
        )
    ])
