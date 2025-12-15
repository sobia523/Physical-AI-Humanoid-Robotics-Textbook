import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from custom_interfaces.action import ManipulateObject

class ManipulationControllerNode(Node):
    def __init__(self):
        super().__init__('manipulation_controller_node')
        self._action_server = ActionServer(
            self,
            ManipulateObject,
            'manipulate_object',
            self.execute_callback)
        self.get_logger().info('Manipulation Controller Node started. Providing action server /manipulate_object.')

    async def execute_callback(self, goal_handle):
        self.get_logger().info(f'Received manipulation goal: {goal_handle.request.action_type} {goal_handle.request.object_id}')

        feedback_msg = ManipulateObject.Feedback()
        feedback_msg.status = f"Starting {goal_handle.request.action_type} for {goal_handle.request.object_id}"
        goal_handle.publish_feedback(feedback_msg)

        # Simulate manipulation
        self.get_logger().info(f'Simulating {goal_handle.request.action_type} of {goal_handle.request.object_id}...')
        await self.get_clock().sleep_for(rclpy.duration.Duration(seconds=2)) # Simulate work

        if goal_handle.request.action_type == "grasp":
            self.get_logger().info(f'Successfully grasped {goal_handle.request.object_id}.')
            result = ManipulateObject.Result()
            result.success = True
            goal_handle.succeed()
            return result
        elif goal_handle.request.action_type == "release":
            self.get_logger().info(f'Successfully released {goal_handle.request.object_id}.')
            result = ManipulateObject.Result()
            result.success = True
            goal_handle.succeed()
            return result
        else:
            self.get_logger().warn(f'Unknown action type: {goal_handle.request.action_type}. Goal failed.')
            result = ManipulateObject.Result()
            result.success = False
            goal_handle.abort()
            return result

def main(args=None):
    rclpy.init(args=args)
    manipulation_controller_node = ManipulationControllerNode()
    rclpy.spin(manipulation_controller_node)
    manipulation_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
