import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import time

class WaypointSender(Node):
    def __init__(self):
        super().__init__('waypoint_sender')
        self.get_logger().info('Waypoint Sender Node started.')
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def send_goal(self, pose):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(f'Sending goal: {goal_msg.pose.pose.position.x}, {goal_msg.pose.pose.position.y}')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.result))
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = WaypointSender()

    # Create a dummy PoseStamped message for the goal
    # In a real scenario, these values would come from user input or a higher-level planner
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map' # Or 'odom', depending on your Nav2 setup
    goal_pose.header.stamp = node.get_clock().now().to_msg()
    goal_pose.pose.position.x = 1.0
    goal_pose.pose.position.y = 0.0
    goal_pose.pose.orientation.w = 1.0 # No rotation

    node.send_goal(goal_pose)

    # Spin until the action is complete
    rclpy.spin(node)

if __name__ == '__main__':
    main()
