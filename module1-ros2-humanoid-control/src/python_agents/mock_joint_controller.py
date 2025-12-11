import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class MockJointController(Node):

    def __init__(self):
        super().__init__('mock_joint_controller')
        self.subscription = self.create_subscription(
            Float64,
            'joint_commands',
            self.joint_command_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.current_joint_position = 0.0
        self.get_logger().info('MockJointController node started, subscribing to /joint_commands')

    def joint_command_callback(self, msg):
        commanded_position = msg.data
        self.current_joint_position = commanded_position # Simply update the "position"
        self.get_logger().info(f'Received joint command: {commanded_position:.2f}. Mock joint position set to: {self.current_joint_position:.2f}')

def main(args=None):
    rclpy.init(args=args)
    mock_joint_controller = MockJointController()
    rclpy.spin(mock_joint_controller)
    mock_joint_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
