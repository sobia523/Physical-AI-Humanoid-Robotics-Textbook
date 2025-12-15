import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarObstacleDetector(Node):

    def __init__(self):
        super().__init__('lidar_obstacle_detector')
        self.subscription = self.create_subscription(
            LaserScan,
            '/demo/scan',  # Topic name as defined in URDF plugin
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.min_distance = 0.5 # meters, threshold for obstacle detection
        self.get_logger().info('LiDAR Obstacle Detector Node has been started.')

    def listener_callback(self, msg):
        # Find the minimum range in the scan
        # Filter out 'inf' (infinity) values which represent no obstacle in range
        finite_ranges = [r for r in msg.ranges if r != float('inf')]
        
        if not finite_ranges:
            self.get_logger().info('No obstacles detected in LiDAR range.')
            return

        closest_distance = min(finite_ranges)

        if closest_distance < self.min_distance:
            self.get_logger().warn(f'OBSTACLE DETECTED! Closest distance: {closest_distance:.2f} m')
        else:
            self.get_logger().info(f'No immediate obstacles. Closest: {closest_distance:.2f} m')

def main(args=None):
    rclpy.init(args=args)
    lidar_obstacle_detector = LidarObstacleDetector()
    rclpy.spin(lidar_obstacle_detector)
    lidar_obstacle_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
