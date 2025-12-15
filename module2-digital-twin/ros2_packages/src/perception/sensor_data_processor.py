import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image, PointCloud2
from cv_bridge import CvBridge
import cv2
import numpy as np

class SensorDataProcessor(Node):

    def __init__(self):
        super().__init__('sensor_data_processor')
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu_sensor', # Replace with actual IMU topic from URDF
            self.imu_callback,
            10)
        self.imu_subscription # prevent unused variable warning

        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw', # Replace with actual camera image topic from URDF
            self.image_callback,
            10)
        self.image_subscription # prevent unused variable warning

        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw', # Replace with actual depth image topic from URDF
            self.depth_callback,
            10)
        self.depth_subscription # prevent unused variable warning

        self.points_subscription = self.create_subscription(
            PointCloud2,
            '/camera/points', # Replace with actual point cloud topic from URDF
            self.points_callback,
            10)
        self.points_subscription # prevent unused variable warning

        self.br = CvBridge()
        self.get_logger().info('Sensor Data Processor Node has been started.')

    def imu_callback(self, msg):
        # Process IMU data
        orientation = msg.orientation
        angular_velocity = msg.angular_velocity
        linear_acceleration = msg.linear_acceleration
        self.get_logger().info(
            f'IMU: Orientation (x,y,z,w): ({orientation.x:.2f}, {orientation.y:.2f}, {orientation.z:.2f}, {orientation.w:.2f}), '
            f'Angular Vel (x,y,z): ({angular_velocity.x:.2f}, {angular_velocity.y:.2f}, {angular_velocity.z:.2f}), '
            f'Linear Accel (x,y,z): ({linear_acceleration.x:.2f}, {linear_acceleration.y:.2f}, {linear_acceleration.z:.2f})'
        )

    def image_callback(self, msg):
        self.get_logger().info('Receiving image frame')
        current_frame = self.br.imgmsg_to_cv2(msg, "bgr8")
        # Display image (optional, requires OpenCV GUI support)
        # cv2.imshow("Camera Feed", current_frame)
        # cv2.waitKey(1)

    def depth_callback(self, msg):
        self.get_logger().info('Receiving depth image frame')
        # Depth image is typically 16UC1 or 32FC1
        depth_frame = self.br.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        # Example: Find min/max depth
        min_depth = np.min(depth_frame[depth_frame > 0]) # Ignore 0s if present
        max_depth = np.max(depth_frame)
        self.get_logger().info(f'Depth: Min: {min_depth:.2f}, Max: {max_depth:.2f}')

    def points_callback(self, msg):
        self.get_logger().info(f'Receiving PointCloud2 with {msg.width * msg.height} points')
        # For actual processing, you'd typically convert to numpy array using sensor_msgs_py.point_cloud2.read_points
        # or PCL libraries. For now, just logging receipt.

def main(args=None):
    rclpy.init(args=args)
    sensor_data_processor = SensorDataProcessor()
    rclpy.spin(sensor_data_processor)
    sensor_data_processor.destroy_node()
    rclpy.shutdown()
    # cv2.destroyAllWindows() # If imshow was used

if __name__ == '__main__':
    main()
