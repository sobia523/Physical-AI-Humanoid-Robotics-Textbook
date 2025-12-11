import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class MockJointController(Node):
    def __init__(self):
        super().__init__('mock_joint_controller')
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.subscription = self.create_subscription(
            Float64,
            'joint_command',
            self.joint_command_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.current_position = 0.0

        # Publish joint states periodically
        self.timer = self.create_timer(0.1, self.publish_joint_states)

    def joint_command_callback(self, msg):
        self.get_logger().info(f'Received joint command: "{msg.data}"')
        self.current_position = msg.data

    def publish_joint_states(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['shoulder_yaw_joint', 'elbow_pitch_joint', 'wrist_roll_joint']
        joint_state_msg.position = [self.current_position, self.current_position, self.current_position]
        self.joint_state_publisher.publish(joint_state_msg)

def main(args=None):
    rclpy.init(args=args)
    mock_joint_controller = MockJointController()
    rclpy.spin(mock_joint_controller)
    mock_joint_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
