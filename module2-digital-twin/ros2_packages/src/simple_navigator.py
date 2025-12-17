import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SimpleNavigator(Node):
    def __init__(self):
        super().__init__('simple_navigator')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('Simple Navigator Node Started. Publishing to /cmd_vel topic.')
        self.timer = self.create_timer(1.0, self.timer_callback) # Publish every 1 second
        self.i = 0

    def timer_callback(self):
        twist_msg = Twist()
        if self.i < 5: # Move forward for 5 seconds
            twist_msg.linear.x = 0.1 # m/s
            self.get_logger().info('Moving Forward')
        elif self.i < 10: # Turn for 5 seconds
            twist_msg.angular.z = 0.2 # rad/s
            self.get_logger().info('Turning')
        else: # Stop and reset
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.get_logger().info('Stopping')
            self.i = 0 # Reset for continuous movement demonstration
            
        self.publisher_.publish(twist_msg)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    simple_navigator = SimpleNavigator()
    rclpy.spin(simple_navigator)
    simple_navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
