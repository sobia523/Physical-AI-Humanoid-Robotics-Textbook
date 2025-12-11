import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from module1_ros2_humanoid_control.action import MoveArm
import sys

class ArmActionClient(Node):

    def __init__(self):
        super().__init__('arm_action_client')
        self._action_client = ActionClient(self, MoveArm, 'move_arm')
        self.get_logger().info('Action client created.')

    def send_goal(self, target_position):
        goal_msg = MoveArm.Goal()
        goal_msg.target_position = float(target_position)

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
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
        self.get_logger().info(f'Result: final_position={result.final_position:.2f}, success={result.success}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Received feedback: current_position={feedback_msg.feedback.current_position:.2f}')

def main(args=None):
    rclpy.init(args=args)
    arm_action_client = ArmActionClient()

    if len(sys.argv) != 2:
        arm_action_client.get_logger().info('Usage: ros2 run module1_ros2_humanoid_control arm_action_client <target_position>')
        return

    target_pos = sys.argv[1]
    arm_action_client.send_goal(target_pos)

    # Spin the node to process the goal, feedback, and result
    executor = SingleThreadedExecutor()
    executor.add_node(arm_action_client)
    executor.spin() # This will block until rclpy.shutdown() is called in get_result_callback

    arm_action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
