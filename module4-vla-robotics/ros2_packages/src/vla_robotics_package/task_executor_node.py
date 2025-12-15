import rclpy
from rclpy.node import Node
from custom_interfaces.msg import TaskPlan, Action
from custom_interfaces.action import ManipulateObject
from geometry_msgs.msg import PoseStamped # For Nav2 NavigateToPose action
from rclpy.action import ActionClient
# from nav2_msgs.action import NavigateToPose # Requires nav2_msgs

class TaskExecutorNode(Node):
    def __init__(self):
        super().__init__('task_executor_node')
        self.subscription = self.create_subscription(
            TaskPlan,
            'task_plan',
            self.task_plan_listener_callback,
            10)
        self.subscription # prevent unused variable warning
        self.get_logger().info('Task Executor Node started. Subscribing to /task_plan.')

        # Action clients (placeholders)
        self._nav_to_pose_action_client = ActionClient(self, None, 'navigate_to_pose') # Replace None with NavigateToPose
        self._manipulate_object_action_client = ActionClient(self, ManipulateObject, 'manipulate_object')

    def task_plan_listener_callback(self, msg):
        self.get_logger().info(f'Received TaskPlan for command: "{msg.source_command}" with {len(msg.actions)} actions.')
        self.execute_task_plan(msg)

    def execute_task_plan(self, task_plan: TaskPlan):
        self.get_logger().info(f'Executing Task Plan ID: {task_plan.id}')
        for action in task_plan.actions:
            self.get_logger().info(f'Processing action: {action.type}')
            if action.type == "Navigate":
                self._execute_navigate_action(action)
            elif action.type == "Grasp":
                self._execute_manipulate_action(action, "grasp")
            elif action.type == "Release":
                self._execute_manipulate_action(action, "release")
            else:
                self.get_logger().warn(f'Unknown action type: {action.type}')

    def _execute_navigate_action(self, action: Action):
        self.get_logger().info(f'Simulating navigation to target: {action.parameters_values}')
        # In a real scenario, send goal to Nav2's navigate_to_pose action server
        # goal_msg = NavigateToPose.Goal()
        # goal_msg.pose = PoseStamped()
        # # Fill pose from action.parameters_values
        # self._nav_to_pose_action_client.wait_for_server()
        # future = self._nav_to_pose_action_client.send_goal_async(goal_msg)
        # rclpy.spin_until_future_complete(self, future)
        # goal_handle = future.result()
        # # Further logic to wait for result and handle feedback

    def _execute_manipulate_action(self, action: Action, action_type: str):
        self.get_logger().info(f'Simulating {action_type} for object: {action.parameters_values}')
        # In a real scenario, send goal to manipulate_object action server
        # goal_msg = ManipulateObject.Goal()
        # goal_msg.action_type = action_type
        # goal_msg.object_id = action.parameters_values[0] if action.parameters_values else ""
        # # Fill target_pose if available in action.parameters_values
        # self._manipulate_object_action_client.wait_for_server()
        # future = self._manipulate_object_action_client.send_goal_async(goal_msg)
        # rclpy.spin_until_future_complete(self, future)
        # goal_handle = future.result()
        # # Further logic to wait for result and handle feedback

def main(args=None):
    rclpy.init(args=args)
    task_executor_node = TaskExecutorNode()
    rclpy.spin(task_executor_node)
    task_executor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
