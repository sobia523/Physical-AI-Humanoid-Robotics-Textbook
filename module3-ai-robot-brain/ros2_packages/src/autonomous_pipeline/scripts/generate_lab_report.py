# This script is a placeholder for generating lab report artifacts.
# It will include mechanisms for collecting maps, sensor data plots, and trajectory visualizations.
# Detailed implementation will depend on how the data is collected and stored during the mission.

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import PointCloud2
# from sensor_msgs.msg import Image # If we want to capture camera images as part of report
import matplotlib.pyplot as plt
import numpy as np
import os
import time

class LabReportGenerator(Node):
    def __init__(self):
        super().__init__('lab_report_generator')
        self.get_logger().info('Lab Report Generator Node started.')

        # Subscribers to collect data from the autonomous mission
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map', # Assuming Nav2 publishes a map
            self.map_callback,
            10
        )
        self.path_subscription = self.create_subscription(
            Path,
            '/vslam/path', # Trajectory from VSLAM
            self.path_callback,
            10
        )
        self.pointcloud_subscription = self.create_subscription(
            PointCloud2,
            '/vslam/map_points', # Point cloud map from VSLAM
            self.pointcloud_callback,
            10
        )
        # Add subscribers for other sensor data if needed for plots
        # self.lidar_subscription = self.create_subscription(...)

        self.occupancy_grid_data = None
        self.robot_path_data = None
        self.point_cloud_data = None
        self.output_dir = "lab_report_artifacts"
        os.makedirs(self.output_dir, exist_ok=True)

    def map_callback(self, msg):
        self.occupancy_grid_data = msg
        self.get_logger().info("Received Occupancy Grid Map.")

    def path_callback(self, msg):
        self.robot_path_data = msg
        self.get_logger().info("Received Robot Path.")

    def pointcloud_callback(self, msg):
        # PointCloud2 message needs to be converted to a more usable format (e.g., numpy array)
        # This is a complex step, just storing the msg for now.
        self.point_cloud_data = msg
        self.get_logger().info("Received Point Cloud Map.")

    def generate_report(self):
        self.get_logger().info("Generating lab report artifacts...")

        if self.occupancy_grid_data:
            self._plot_occupancy_grid(self.occupancy_grid_data)
        if self.robot_path_data:
            self._plot_trajectory(self.robot_path_data)
        if self.point_cloud_data:
            self.get_logger().info("Point cloud visualization requires advanced processing, skipping for placeholder.")
            # self._plot_point_cloud(self.point_cloud_data)

        self.get_logger().info(f"Lab report artifacts generated in {self.output_dir}")

    def _plot_occupancy_grid(self, occupancy_grid_msg):
        width = occupancy_grid_msg.info.width
        height = occupancy_grid_msg.info.height
        resolution = occupancy_grid_msg.info.resolution
        origin_x = occupancy_grid_msg.info.origin.position.x
        origin_y = occupancy_grid_msg.info.origin.position.y

        # Convert 1D occupancy grid data to 2D numpy array
        grid_data = np.array(occupancy_grid_msg.data).reshape((height, width))
        grid_data_flipped = np.flipud(grid_data) # Flip because matplotlib origin is bottom-left

        plt.figure(figsize=(10, 10))
        plt.imshow(grid_data_flipped, cmap='gray', origin='lower',
                   extent=[origin_x, origin_x + width * resolution,
                           origin_y, origin_y + height * resolution])
        plt.title('Occupancy Grid Map')
        plt.xlabel('X position (m)')
        plt.ylabel('Y position (m)')
        plt.colorbar(label='Occupancy Probability (-1:unknown, 0:free, 100:occupied)')
        plt.savefig(os.path.join(self.output_dir, 'occupancy_grid_map.png'))
        plt.close()
        self.get_logger().info("Generated Occupancy Grid Map plot.")

    def _plot_trajectory(self, path_msg):
        x_coords = [pose_stamped.pose.position.x for pose_stamped in path_msg.poses]
        y_coords = [pose_stamped.pose.position.y for pose_stamped in path_msg.poses]

        plt.figure(figsize=(10, 10))
        plt.plot(x_coords, y_coords, marker='o', linestyle='-', color='blue')
        plt.title('Robot Trajectory')
        plt.xlabel('X position (m)')
        plt.ylabel('Y position (m)')
        plt.grid(True)
        plt.savefig(os.path.join(self.output_dir, 'robot_trajectory.png'))
        plt.close()
        self.get_logger().info("Generated Robot Trajectory plot.")

    # def _plot_point_cloud(self, point_cloud_msg):
    #     """
    #     Placeholder for point cloud plotting. Requires more complex parsing of PointCloud2 msg.
    #     """
    #     self.get_logger().warn("Point cloud plotting is not fully implemented in this placeholder.")
    #     pass

def main(args=None):
    rclpy.init(args=args)
    generator = LabReportGenerator()

    # Spin the node for a duration to collect data, then generate report
    # In a real scenario, this would be triggered after a mission is complete.
    generator.get_logger().info("Collecting data for 10 seconds...")
    end_time = time.time() + 10
    while rclpy.ok() and time.time() < end_time:
        rclpy.spin_once(generator, timeout_sec=0.1)
    
    generator.generate_report()
    generator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
