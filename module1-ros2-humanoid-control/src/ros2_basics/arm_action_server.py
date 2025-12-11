import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from module1_ros2_humanoid_control.action import MoveArm
import time

class ArmActionServer(Node):

    def __init__(self):
        super().__init__('arm_action_server')
        self._action_server = ActionServer(
            self,
            MoveArm,
            'move_arm',
            self.execute_callback)
        self.get_logger().info('Action server ready.')

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Executing goal: target_position={goal_handle.request.target_position}')

        feedback_msg = MoveArm.Feedback()
        feedback_msg.current_position = 0.0 # Start from 0 for simplicity

        # Simulate arm movement
        distance_to_move = goal_handle.request.target_position - feedback_msg.current_position
        step = 0.1 * (1 if distance_to_move > 0 else -1) # Move in 0.1 increments towards target

        while abs(goal_handle.request.target_position - feedback_msg.current_position) > abs(step / 2):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled!')
                return MoveArm.Result(final_position=feedback_msg.current_position, success=False)

            feedback_msg.current_position += step
            self.get_logger().info(f'Feedback: current_position={feedback_msg.current_position:.2f}')
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5) # Simulate movement time

        goal_handle.succeed()
        result = MoveArm.Result()
        result.final_position = goal_handle.request.target_position
        result.success = True
        self.get_logger().info(f'Goal succeeded! Final position: {result.final_position:.2f}')
        return result

def main(args=None):
    rclpy.init(args=args)
    arm_action_server = ArmActionServer()
    rclpy.spin(arm_action_server)
    arm_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
