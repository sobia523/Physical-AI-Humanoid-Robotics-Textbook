import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time

class SimpleAgent(Node):

    def __init__(self):
        super().__init__('simple_agent')
        self.publisher_ = self.create_publisher(Float64, 'joint_commands', 10)
        self.timer_period = 1.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.joint_commands = [0.0, 0.5, 1.0, 0.5, 0.0, -0.5, -1.0, -0.5] # Example sequence of joint positions
        self.command_index = 0
        self.get_logger().info('SimpleAgent node started, publishing to /joint_commands')

    def timer_callback(self):
        msg = Float64()
        msg.data = self.joint_commands[self.command_index]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Sending joint command: {msg.data:.2f}')
        
        self.command_index = (self.command_index + 1) % len(self.joint_commands)

def main(args=None):
    rclpy.init(args=args)
    simple_agent = SimpleAgent()
    rclpy.spin(simple_agent)
    simple_agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
