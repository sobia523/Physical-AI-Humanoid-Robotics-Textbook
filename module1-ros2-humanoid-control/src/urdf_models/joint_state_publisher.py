import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math

class SimpleJointStatePublisher(Node):

    def __init__(self):
        super().__init__('simple_joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.timer_callback) # Publish at 10 Hz
        self.i = 0
        self.get_logger().info('SimpleJointStatePublisher node started.')

    def timer_callback(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['shoulder_pitch_joint', 'elbow_yaw_joint'] # Names from URDF
        
        # Animate joints in a simple sine wave
        msg.position = [
            math.sin(self.i * 0.05), # shoulder_pitch_joint
            math.cos(self.i * 0.05)  # elbow_yaw_joint
        ]
        msg.velocity = [] # Can be left empty for position-only control
        msg.effort = []   # Can be left empty for position-only control

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing joint states: {msg.position}')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    simple_joint_state_publisher = SimpleJointStatePublisher()
    rclpy.spin(simple_joint_state_publisher)
    simple_joint_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
