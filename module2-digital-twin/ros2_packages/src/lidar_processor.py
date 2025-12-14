import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.get_logger().info('LiDAR Processor Node Started. Subscribing to /scan topic.')

    def listener_callback(self, msg):
        # Basic obstacle detection: check if any range measurement is below a threshold
        obstacle_detected = False
        obstacle_distance = float('inf')
        
        # Define a safe distance threshold (e.g., 0.5 meters)
        SAFE_DISTANCE = 0.5

        for r in msg.ranges:
            if msg.range_min < r < SAFE_DISTANCE:
                obstacle_detected = True
                obstacle_distance = min(obstacle_distance, r)
        
        if obstacle_detected:
            self.get_logger().warn(f'OBSTACLE DETECTED! Nearest obstacle at {obstacle_distance:.2f} meters.')
        # else:
        #     self.get_logger().info('No obstacle detected in range.') # Too chatty for frequent updates

def main(args=None):
    rclpy.init(args=args)
    lidar_processor = LidarProcessor()
    rclpy.spin(lidar_processor)
    lidar_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
