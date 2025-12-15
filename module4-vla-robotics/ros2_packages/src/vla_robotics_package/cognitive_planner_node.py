import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from custom_interfaces.msg import TaskPlan, Action
from custom_interfaces.srv import GenerateTaskPlan
# import openai # OpenAI Python client will need to be installed

class CognitivePlannerNode(Node):
    def __init__(self):
        super().__init__('cognitive_planner_node')
        self.subscription = self.create_subscription(
            String,
            'transcribed_text',
            self.transcribed_text_listener_callback,
            10)
        self.subscription # prevent unused variable warning
        self.publisher_ = self.create_publisher(TaskPlan, 'task_plan', 10)
        self.srv = self.create_service(GenerateTaskPlan, 'generate_task_plan', self.generate_task_plan_callback)
        self.get_logger().info('Cognitive Planner Node started. Subscribing to /transcribed_text, Publishing to /task_plan, Providing service /generate_task_plan.')

    def transcribed_text_listener_callback(self, msg):
        self.get_logger().info(f'Received Transcribed Text: "{msg.data}"')
        task_plan = self._generate_plan_from_text(msg.data)
        self.publisher_.publish(task_plan)
        self.get_logger().info('Published TaskPlan from transcribed text.')

    def generate_task_plan_callback(self, request, response):
        self.get_logger().info(f'Received service request for command: "{request.command}"')
        response.plan = self._generate_plan_from_text(request.command)
        self.get_logger().info('Responded with TaskPlan.')
        return response

    def _generate_plan_from_text(self, command_text):
        # Placeholder for OpenAI API call
        # In a real scenario, use OpenAI API to convert command_text into a structured plan.
        # Example:
        # prompt = f"Convert the command '{command_text}' into a sequence of robotics actions."
        # response = openai.chat.completions.create(
        #     model="gpt-3.5-turbo",
        #     messages=[
        #         {"role": "system", "content": "You are a helpful robot task planner."},
        #         {"role": "user", "content": prompt}
        #     ]
        # )
        # llm_output = response.choices[0].message.content
        # parse llm_output to create TaskPlan and Action messages

        task_plan = TaskPlan()
        task_plan.id = "plan_" + str(self.get_clock().now().nanoseconds)
        task_plan.source_command = command_text
        task_plan.status = "Pending"

        # Dummy actions
        if "pick up" in command_text and "move it to" in command_text:
            action1 = Action()
            action1.type = "Grasp"
            action1.parameters_keys = ["object_id"]
            action1.parameters_values = ["dummy_box"]
            action1.status = "Pending"
            task_plan.actions.append(action1)

            action2 = Action()
            action2.type = "Navigate"
            action2.parameters_keys = ["target_location"]
            action2.parameters_values = ["table_location"] # Example: "[1.0, 2.0, 0.0]"
            action2.status = "Pending"
            task_plan.actions.append(action2)

            action3 = Action()
            action3.type = "Release"
            action3.parameters_keys = ["object_id"]
            action3.parameters_values = ["dummy_box"]
            action3.status = "Pending"
            task_plan.actions.append(action3)
        else:
            action1 = Action()
            action1.type = "UnknownCommand"
            action1.parameters_keys = []
            action1.parameters_values = []
            action1.status = "Failed"
            task_plan.actions.append(action1)

        return task_plan

def main(args=None):
    rclpy.init(args=args)
    cognitive_planner_node = CognitivePlannerNode()
    rclpy.spin(cognitive_planner_node)
    cognitive_planner_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
