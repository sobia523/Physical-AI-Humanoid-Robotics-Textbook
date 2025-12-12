# This script is designed for executing an autonomous navigation mission within Isaac Sim,
# leveraging integrated VSLAM and Nav2 components for obstacle avoidance.
#
# Note: This is a conceptual script demonstrating the ROS 2 side of mission execution.
# Actual Isaac Sim control (e.g., spawning obstacles, reading ground truth)
# would typically be handled within Isaac Sim's Python environment, potentially
# through a ROS 2 bridge or direct Omniverse Kit API calls.

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time
import random

class AutonomousMissionExecutor(Node):
    def __init__(self):
        super().__init__('autonomous_mission_executor')
        self.get_logger().info('Autonomous Mission Executor Node started.')
        
        # Create an action client for Nav2's NavigateToPose action
        # Using custom QoS profile to ensure reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose', qos_profile=qos_profile)
        
        self.current_waypoint_index = 0
        self.waypoints = [] # List of PoseStamped objects for the mission

    def add_waypoint(self, x, y, yaw=0.0, frame_id='map'):
        """
        Adds a new waypoint to the mission plan.
        """
        waypoint = PoseStamped()
        waypoint.header.frame_id = frame_id
        waypoint.header.stamp = self.get_clock().now().to_msg()
        waypoint.pose.position.x = float(x)
        waypoint.pose.position.y = float(y)
        # Convert yaw to quaternion (simple 2D navigation)
        from tf_transformations import quaternion_from_euler
        q = quaternion_from_euler(0.0, 0.0, float(yaw))
        waypoint.pose.orientation.x = q[0]
        waypoint.pose.orientation.y = q[1]
        waypoint.pose.orientation.z = q[2]
        waypoint.pose.orientation.w = q[3]
        self.waypoints.append(waypoint)
        self.get_logger().info(f"Waypoint added: ({x}, {y}, {yaw})")

    def start_mission(self):
        """
        Starts executing the mission by sending the first waypoint.
        """
        if not self.waypoints:
            self.get_logger().warn("No waypoints defined for the mission.")
            return

        self.get_logger().info("Starting autonomous navigation mission...")
        self._send_next_waypoint()

    def _send_next_waypoint(self):
        """
        Sends the next waypoint in the list to the Nav2 action server.
        """
        if self.current_waypoint_index < len(self.waypoints):
            waypoint_to_send = self.waypoints[self.current_waypoint_index]
            self.get_logger().info(f"Navigating to waypoint {self.current_waypoint_index + 1}/"
                                   f"{len(self.waypoints)}: x={waypoint_to_send.pose.position.x:.2f}, "
                                   f"y={waypoint_to_send.pose.position.y:.2f}")
            self.send_goal(waypoint_to_send)
        else:
            self.get_logger().info("All waypoints reached. Mission complete!")
            # rclpy.shutdown() # Uncomment to shutdown after all waypoints

    def send_goal(self, pose_stamped_msg):
        self.get_logger().info('Waiting for Nav2 action server...')
        # wait_for_server can block, consider a timeout or running in a separate thread/executor for non-blocking
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2 action server not available!")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped_msg

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2 server :(')
            self._send_next_waypoint() # Try next waypoint, or handle failure
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        
        if status == 4: # GoalStatus.SUCCEEDED
            self.get_logger().info(f'Waypoint {self.current_waypoint_index + 1} reached successfully!')
            self.current_waypoint_index += 1
            self._send_next_waypoint()
        else:
            self.get_logger().error(f'Navigation to waypoint {self.current_waypoint_index + 1} failed with status: {status}')
            # Implement recovery behavior or retry logic here
            self.current_waypoint_index += 1 # For now, just try next waypoint
            self._send_next_waypoint()

    def shutdown(self):
        """
        Shuts down the node gracefully.
        """
        self.get_logger().info("Shutting down Autonomous Mission Executor.")
        self._action_client.destroy()
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    executor = AutonomousMissionExecutor()

    # Define a simple mission path with multiple waypoints
    executor.add_waypoint(x=1.0, y=0.0, yaw=0.0)
    executor.add_waypoint(x=1.0, y=1.0, yaw=1.57) # Turn left
    executor.add_waypoint(x=0.0, y=1.0, yaw=3.14) # Turn around
    executor.add_waypoint(x=0.0, y=0.0, yaw=-1.57) # Return to origin

    # Incorporating obstacle avoidance is handled by Nav2's costmap and planner.
    # To demonstrate obstacle avoidance, obstacles would need to be present
    # in the Isaac Sim environment when the mission is executed.

    executor.start_mission()

    try:
        rclpy.spin(executor)
    except KeyboardInterrupt:
        executor.get_logger().info("Mission interrupted by user.")
    finally:
        executor.shutdown()

if __name__ == '__main__':
    main()