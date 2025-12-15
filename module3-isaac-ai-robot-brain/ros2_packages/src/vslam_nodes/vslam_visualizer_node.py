# Placeholder for ROS 2 VSLAM Visualizer Node (for Rviz)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path # Example messages for visualization

class VslamVisualizerNode(Node):
    def __init__(self):
        super().__init__('vslam_visualizer_node')
        
        # Subscribe to VSLAM pose output
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/vslam/pose', # Topic from VSLAM
            self.pose_callback,
            10)
        self.pose_subscription

        # Subscribe to VSLAM map output
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/vslam/map', # Topic from VSLAM
            self.map_callback,
            10)
        self.map_subscription

        self.path_publisher = self.create_publisher(Path, '/vslam/path', 10)
        self.path = Path()
        self.path.header.frame_id = 'map' # Assuming VSLAM publishes in 'map' frame

        self.get_logger().info('VSLAM Visualizer Node started. Waiting for VSLAM data...')

    def pose_callback(self, msg):
        self.get_logger().info(f'Received VSLAM Pose: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}')
        self.path.poses.append(msg)
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path_publisher.publish(self.path)

    def map_callback(self, msg):
        self.get_logger().info(f'Received VSLAM Map: width={msg.info.width}, height={msg.info.height}')
        # Rviz will display this OccupancyGrid directly if configured correctly.

def main(args=None):
    rclpy.init(args=args)
    vslam_visualizer_node = VslamVisualizerNode()
    rclpy.spin(vslam_visualizer_node)
    vslam_visualizer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
