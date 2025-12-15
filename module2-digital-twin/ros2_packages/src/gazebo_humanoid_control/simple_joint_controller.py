import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Duration

class SimpleJointController(Node):

    def __init__(self):
        super().__init__('simple_joint_controller')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/shoulder_controller/commands', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0.0
        self.direction = 1

    def timer_callback(self):
        msg = Float64MultiArray()
        # Oscillate between -1.57 and 1.57 radians (approx -90 to 90 degrees)
        self.i += self.direction * 0.1
        if self.i > 1.57:
            self.i = 1.57
            self.direction = -1
        elif self.i < -1.57:
            self.i = -1.57
            self.direction = 1
        
        msg.data = [self.i] # Assuming one joint for now
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{self.i}"')

def main(args=None):
    rclpy.init(args=args)
    simple_joint_controller = SimpleJointController()
    rclpy.spin(simple_joint_controller)
    simple_joint_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
