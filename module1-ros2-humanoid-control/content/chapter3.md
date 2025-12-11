# Chapter 3: ROS 2 Services and Actions

## 3.1 Implementing Services and Actions

ROS 2 provides powerful communication mechanisms beyond the publish-subscribe pattern, specifically designed for request-reply interactions and long-running, pre-emptible tasks. These are **Services** and **Actions**, respectively. Understanding when to use each is crucial for designing robust robot control systems.

-   **When to use Services vs. Actions**:
    -   **Services** are best suited for short-duration, blocking operations where a client sends a request and expects an immediate response. Think of it like a function call: you ask for something, and you get an answer right away. Examples include querying a sensor's current value, commanding a gripper to open or close, or setting a robot's configuration.
    -   **Actions** are designed for long-running tasks that provide continuous feedback on their progress and can be cancelled (pre-empted) before completion. Imagine telling a robot to navigate across a room: you want to know if it's making progress (feedback), you might want to tell it to stop halfway (pre-emption), and eventually, you want to know if it reached the destination (result). Actions are more complex but offer greater control for these types of tasks.

-   **Service Definition Structure (.srv)**:
    ROS 2 service definitions are stored in `.srv` files, which define the structure of the request and response messages. A service file is divided into two parts by a `---` separator. The top part defines the request fields, and the bottom part defines the response fields.
    ```
    # Request message fields
    field_type1 field_name1
    field_type2 field_name2
    ---
    # Response message fields
    field_type3 field_name3
    field_type4 field_name4
    ```

-   **Action Definition Structure (.action)**:
    ROS 2 action definitions are stored in `.action` files, which define three types of messages separated by `---`: goal, result, and feedback.
    ```
    # Goal message fields
    field_type1 field_name1
    ---
    # Result message fields
    field_type2 field_name2
    ---
    # Feedback message fields
    field_type3 field_name3
    ```

## 3.2 Client-Server Patterns in ROS 2

### 3.2.1 Services: Request-Reply

Services implement a strict client-server model. A **service server** node provides a service, waiting for requests. A **service client** node sends a request to a service server and waits for a response.

-   **Creating a Service Server (Python)**:
    A Python service server uses `rclpy` to create a service. It defines a callback function that executes when a client sends a request. This function processes the request and populates the response message.
    ```python
    import rclpy
    from rclpy.node import Node
    from example_interfaces.srv import AddTwoInts # Example service type

    class MinimalService(Node):
        def __init__(self):
            super().__init__('minimal_service')
            self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
            self.get_logger().info('Service server ready.')

        def add_two_ints_callback(self, request, response):
            response.sum = request.a + request.b
            self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}. Sending response: {response.sum}')
            return response

    def main(args=None):
        rclpy.init(args=args)
        minimal_service = MinimalService()
        rclpy.spin(minimal_service)
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

-   **Creating a Service Client (Python)**:
    A Python service client also uses `rclpy` to create a client. It waits for the service to be available, creates a request message, sends it, and then waits for a response.
    ```python
    import rclpy
    from rclpy.node import Node
    from example_interfaces.srv import AddTwoInts
    import sys

    class MinimalClientAsync(Node):
        def __init__(self):
            super().__init__('minimal_client_async')
            self.cli = self.create_client(AddTwoInts, 'add_two_ints')
            while not self.cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
            self.req = AddTwoInts.Request()

        def send_request(self, a, b):
            self.req.a = a
            self.req.b = b
            self.future = self.cli.call_async(self.req)
            rclpy.spin_until_future_complete(self, self.future)
            return self.future.result()

    def main(args=None):
        rclpy.init(args=args)
        minimal_client = MinimalClientAsync()
        response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
        minimal_client.get_logger().info(f'Result of add_two_ints: for {sys.argv[1]} + {sys.argv[2]} = {response.sum}')
        minimal_client.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

### 3.2.2 Actions: Goal-Feedback-Result

Actions introduce a more complex interaction pattern with an **action server** and an **action client**.

-   **Creating an Action Server (Python)**:
    An action server receives goals from clients, executes them, sends periodic feedback, and ultimately returns a result. It's often implemented with multiple callback functions for handling goals, pre-emption, and execution.
    ```python
    import rclpy
    from rclpy.action import ActionServer
    from rclpy.node import Node
    from example_interfaces.action import Fibonacci # Example action type
    import time

    class MinimalActionServer(Node):
        def __init__(self):
            super().__init__('minimal_action_server')
            self._action_server = ActionServer(
                self,
                Fibonacci,
                'fibonacci',
                self.execute_callback)
            self.get_logger().info('Action server ready.')

        def execute_callback(self, goal_handle):
            self.get_logger().info(f'Executing goal: {goal_handle.request.order}')

            feedback_msg = Fibonacci.Feedback()
            feedback_msg.sequence = [0, 1]

            for i in range(1, goal_handle.request.order):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('Goal canceled!')
                    return Fibonacci.Result()

                feedback_msg.sequence.append(feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
                self.get_logger().info(f'Feedback: {feedback_msg.sequence}')
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(1)

            goal_handle.succeed()
            result = Fibonacci.Result()
            result.sequence = feedback_msg.sequence
            self.get_logger().info(f'Goal succeeded! Result: {result.sequence}')
            return result

    def main(args=None):
        rclpy.init(args=args)
        minimal_action_server = MinimalActionServer()
        rclpy.spin(minimal_action_server)
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

-   **Creating an Action Client (Python)**:
    An action client sends a goal to an action server, can monitor the feedback, and awaits the final result. It can also send a cancellation request.
    ```python
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from example_interfaces.action import Fibonacci
    import time

    class MinimalActionClient(Node):
        def __init__(self):
            super().__init__('minimal_action_client')
            self._action_client = ActionClient(self, Fibonacci, 'fibonacci')
            self.get_logger().info('Action client created.')

        def send_goal(self, order):
            goal_msg = Fibonacci.Goal()
            goal_msg.order = order

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
            self.get_logger().info(f'Result: {result.sequence}')
            rclpy.shutdown()

        def feedback_callback(self, feedback_msg):
            self.get_logger().info(f'Received feedback: {feedback_msg.feedback.sequence}')

    def main(args=None):
        rclpy.init(args=args)
        action_client = MinimalActionClient()
        action_client.send_goal(10)
        rclpy.spin(action_client)

    if __name__ == '__main__':
        main()
    ```

## 3.3 Example: Robot Command-Response Service

This section will detail a practical example of a custom ROS 2 service. We will define a service `SetGripperState` that takes a boolean (open/close) as a request and returns a boolean indicating success.

### Running the Example

To run the `gripper_service_server.py` and `gripper_service_client.py` example, you will first need to build your ROS 2 workspace to generate the custom service messages. Navigate to the root of your `module1-ros2-humanoid-control` directory and execute:

```bash
colcon build --symlink-install
```

After a successful build, source your workspace to make the new packages available:

```bash
source install/setup.bash
```

Now, you can run the service server in one terminal:

**Terminal 1 (Service Server):**
```bash
ros2 run module1_ros2_humanoid_control gripper_service_server
```

And then run the client in another terminal, providing the desired gripper state (`true` to open, `false` to close):

**Terminal 2 (Service Client - Open Gripper):**
```bash
ros2 run module1_ros2_humanoid_control gripper_service_client true
```

**Terminal 2 (Service Client - Close Gripper):**
```bash
ros2 run module1_ros2_humanoid_control gripper_service_client false
```

You should see output in the server terminal indicating the gripper state change, and the client terminal confirming the service call success.

## 3.4 Example: Long-Running Robot Task Action

This section will provide a practical example of a custom ROS 2 action. We will define an action `MoveArm` that takes a target joint position as a goal, provides continuous feedback on the current joint position, and returns the final achieved position as a result.

### Running the Example

To run the `arm_action_server.py` and `arm_action_client.py` example, you will first need to ensure your ROS 2 workspace is built to generate the custom action messages. Navigate to the root of your `module1-ros2-humanoid-control` directory and execute:

```bash
colcon build --symlink-install
```

After a successful build, source your workspace to make the new packages available:

```bash
source install/setup.bash
```

Now, you can run the action server in one terminal:

**Terminal 1 (Action Server):**
```bash
ros2 run module1_ros2_humanoid_control arm_action_server
```

And then run the client in another terminal, providing the desired target position for the arm:

**Terminal 2 (Action Client - Move arm to 2.0):**
```bash
ros2 run module1_ros2_humanoid_control arm_action_client 2.0
```

You should observe the action server printing feedback messages indicating the arm's current position as it moves towards the target, and finally, the client will receive the result once the action is complete.

## 3.5 Best Practices for Services and Actions

Designing and implementing services and actions effectively requires careful consideration of their unique characteristics.

### Designing Clear Service/Action Interfaces

-   **Specificity**: Define service/action types that are narrowly focused on a single logical operation or task. Avoid overly broad interfaces.
-   **Intuitiveness**: The request/goal and response/result messages should be easy to understand and use by other nodes.
-   **Completeness**: Ensure that all necessary data for the operation (inputs) and all expected outcomes (outputs, feedback) are included in the message definitions.

### Error Handling and Timeouts

-   **Services**: Clients should implement robust error handling for service calls, including timeouts (`wait_for_service`) and checking the success status of the response. Servers should return meaningful error indications in their response messages.
-   **Actions**: Clients need to manage the asynchronous nature of actions, including handling goal rejections, cancellation, and monitoring for server failures. Servers should gracefully handle pre-emption requests and ensure that feedback and results are consistent.

### Pre-emption in Actions

A key advantage of actions is their support for pre-emption. An action client can send a cancellation request to an action server, instructing it to stop its current task. Action servers must be designed to periodically check for cancellation requests and gracefully terminate their execution, ensuring the robot can safely stop or transition to another task. This is crucial for reactive robot behaviors and safety.
