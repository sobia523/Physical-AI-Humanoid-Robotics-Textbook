# Placeholder for ROS 2 RGB-D Data Capture Node

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo # For RGB and Depth
from cv_bridge import CvBridge # Often used for converting ROS Image messages to OpenCV images

class RgbdCaptureNode(Node):
    def __init__(self):
        super().__init__('rgbd_capture_node')
        self.bridge = CvBridge()

        self.rgb_subscription = self.create_subscription(
            Image,
            '/camera/rgb/image_raw', # Topic for RGB image from Isaac Sim
            self.rgb_callback,
            10)
        self.rgb_subscription

        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw', # Topic for Depth image from Isaac Sim
            self.depth_callback,
            10)
        self.depth_subscription
        
        self.get_logger().info('RGB-D Capture Node started. Waiting for data...')

    def rgb_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.get_logger().info(f'Received RGB Image: shape={cv_image.shape}')
            # Further processing or storage can happen here
        except Exception as e:
            self.get_logger().error(f'Error processing RGB image: {e}')

    def depth_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, msg.encoding) # Depth encoding varies (e.g., 32FC1, 16UC1)
            self.get_logger().info(f'Received Depth Image: shape={cv_image.shape}')
            # Further processing or storage can happen here
        except Exception as e:
            self.get_logger().error(f'Error processing Depth image: {e}')

def main(args=None):
    rclpy.init(args=args)
    rgbd_capture_node = RgbdCaptureNode()
    rclpy.spin(rgbd_capture_node)
    rgbd_capture_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
