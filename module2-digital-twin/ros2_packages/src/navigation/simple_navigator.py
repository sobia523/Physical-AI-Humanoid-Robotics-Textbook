import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class SimpleNavigator(Node):

    def __init__(self):
        super().__init__('simple_navigator')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/demo/scan', # Topic from LiDAR sensor in URDF/Gazebo
            self.scan_callback,
            10)
        self.scan_subscriber # prevent unused variable warning

        self.twist_msg = Twist()
        self.obstacle_detected = False
        self.get_logger().info('Simple Navigator Node has been started.')

    def scan_callback(self, msg):
        # Very simple obstacle avoidance: if anything is too close in front, stop and turn
        min_front_distance = 1.0 # meters
        
        # Consider a front sector (e.g., -30 to +30 degrees)
        # Assuming msg.angle_min is -pi/2 (-90 deg) and angle_max is pi/2 (90 deg) for 180 deg scan
        # Need to adjust this based on actual LiDAR configuration
        
        # For simplicity, just check a few points in the front
        num_ranges = len(msg.ranges)
        if num_ranges == 0:
            self.obstacle_detected = False
            return

        # Check front sector (adjust indices based on your LiDAR's number of samples and angular range)
        # This example assumes a 180-degree scan where middle is front
        front_indices = [i for i in range(num_ranges) if msg.angle_min + i * msg.angle_increment >= -math.pi/6 and msg.angle_min + i * msg.angle_increment <= math.pi/6]
        
        closest_distance_in_front = float('inf')
        for i in front_indices:
            if not math.isinf(msg.ranges[i]):
                closest_distance_in_front = min(closest_distance_in_front, msg.ranges[i])

        if closest_distance_in_front < min_front_distance:
            self.obstacle_detected = True
            self.get_logger().warn(f'Front obstacle too close: {closest_distance_in_front:.2f} m')
        else:
            self.obstacle_detected = False
            
        self.publish_cmd_vel()

    def publish_cmd_vel(self):
        if self.obstacle_detected:
            # Stop and turn
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = 0.5 # Turn left
            self.get_logger().info('Turning to avoid obstacle.')
        else:
            # Move forward slowly
            self.twist_msg.linear.x = 0.2
            self.twist_msg.angular.z = 0.0
            self.get_logger().info('Moving forward.')
            
        self.cmd_vel_publisher.publish(self.twist_msg)

def main(args=None):
    rclpy.init(args=args)
    simple_navigator = SimpleNavigator()
    rclpy.spin(simple_navigator)
    simple_navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
