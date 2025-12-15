# Chapter 2: ROS 2 Nodes and Topics

## 2.1 Creating ROS 2 Nodes in Python
In ROS 2, Python is a popular choice for developing robot applications due to its readability and extensive libraries. The `rclpy` client library provides the necessary tools to interface Python programs with the ROS 2 graph. At the heart of every ROS 2 Python application is the Node.

A basic ROS 2 Python node follows a common structure:
1.  **Import `rclpy` and `Node`**: These are the essential components from the `rclpy` library. `rclpy` is the Python client library for ROS 2, enabling Python applications to communicate with the ROS 2 system. The `Node` class is the base class for all user-defined ROS 2 nodes in Python.
2.  **Initialize `rclpy`**: Before any ROS 2 communication can occur, the `rclpy.init(args=args)` function must be called. This initializes the ROS 2 client library, sets up signal handlers, and performs other necessary background tasks. The `args` parameter allows the passing of command-line arguments to the ROS 2 system, which can be useful for node remapping or other configurations.
3.  **Create a Node**: Instantiate a `Node` object, typically by creating a class that inherits from `rclpy.node.Node`. When you call `super().__init__('my_node_name')`, you are initializing the base `Node` class and providing a unique name for your node within the ROS 2 graph. This name is crucial for identification and communication.
4.  **Spin the Node**: The `rclpy.spin(node)` function is the entry point for a node's event loop. It keeps the node alive, allowing it to process callbacks from publishers, subscribers, services, and actions. This function is blocking, meaning it will run indefinitely until the node is shut down (e.g., by a `Ctrl+C` signal, or explicitly by `rclpy.shutdown()`). It's crucial for event-driven ROS 2 applications to continuously check for incoming messages or service requests.
5.  **Shutdown `rclpy`**: Once the node's work is done (e.g., on program exit or `Ctrl+C`), `rclpy.shutdown()` should be called to cleanly de-initialize the ROS 2 client library and release any allocated resources. Before shutting down `rclpy`, it's good practice to `destroy_node()` to properly clean up the individual node's resources.

```python
import rclpy
from rclpy.node import Node

class MyCustomNode(Node):
    def __init__(self):
        super().__init__('my_custom_node') # Initialize the node with a unique name
        self.get_logger().info('MyCustomNode has been started!')
        # You can declare parameters here
        self.declare_parameter('my_parameter', 'world')

        # And retrieve them
        my_param = self.get_parameter('my_parameter').get_parameter_value().string_value
        self.get_logger().info(f'Hello {my_param}!')


def main(args=None):
    rclpy.init(args=args) # Initialize ROS 2 client library
    node = MyCustomNode()  # Create the node
    rclpy.spin(node)       # Keep the node alive and processing events
    node.destroy_node()    # Destroy the node cleanly
    rclpy.shutdown()       # Shutdown ROS 2 client library

if __name__ == '__main__':
    main()
```
This fundamental structure forms the basis for all ROS 2 Python nodes, regardless of whether they publish, subscribe, or provide services/actions. The ability to declare and use parameters within a node, as shown in the example, adds another layer of configurability and flexibility, allowing the node's behavior to be modified without recompiling code. Parameters can be set from launch files, the command line, or dynamically at runtime.

### Node Lifecycle and Management

Beyond basic creation, understanding the lifecycle of a ROS 2 node is important. Nodes can be managed through various tools:

*   **Command Line Tools**: `ros2 run` is used to execute a single ROS 2 executable (a node). `ros2 launch` is used to start multiple nodes and configurations defined in a launch file, which provides a more structured way to manage complex applications.
*   **Launch Files**: Written in Python or XML, launch files allow for automated startup, configuration, and orchestration of multiple nodes. They can pass parameters, remap topics, and define group actions.
*   **Introspection**: Tools like `ros2 node list` and `ros2 node info <node_name>` allow you to inspect running nodes and their connections, which is invaluable for debugging.

The graceful shutdown of nodes is critical for resource management. When a node is destroyed, it unregisters itself from the ROS 2 graph, closes communication channels, and frees memory. Improper shutdown can lead to orphaned resources or communication issues.

The `Node` class in `rclpy` offers methods to interact with the ROS 2 graph, such as `create_publisher`, `create_subscription`, `create_service`, `create_client`, `create_action_server`, and `create_action_client`. It also provides access to logging facilities (`self.get_logger()`) and parameter management (`self.declare_parameter()`, `self.get_parameter()`). These methods are the building blocks for creating sophisticated ROS 2 applications in Python.

## 2.2 Publishing and Subscribing to Topics

Topics are the primary mechanism for asynchronous, streaming data exchange in ROS 2. They are fundamental for disseminating sensor data, status updates, and other continuous information throughout the robot system.

### 2.2.1 Publishers

A **Publisher** is a component within a node responsible for sending messages to a specific topic. To create a publisher, a node uses its `create_publisher` method, specifying the message type and the topic name.

```python
from std_msgs.msg import String # Example message type

# Inside your node's __init__ method
self.publisher_ = self.create_publisher(String, 'chatter', 10) # String message type, topic name 'chatter', QoS depth 10
timer_period = 0.5  # seconds
self.timer = self.create_timer(timer_period, self.timer_callback)
self.i = 0

def timer_callback(self):
    msg = String()
    msg.data = 'Hello ROS 2: %d' % self.i
    self.publisher_.publish(msg)
    self.get_logger().info('Publishing: "%s"' % msg.data)
    self.i += 1
```
Publishing messages typically happens periodically using a timer callback or in response to an event. The `rclpy` timer mechanism allows you to execute a function at a fixed rate, ensuring a steady stream of data if needed.

### 2.2.2 Subscribers

A **Subscriber** is a component within a node that receives messages from a specific topic. To create a subscriber, a node uses its `create_subscription` method, providing the message type, topic name, and a callback function that will be executed whenever a new message arrives.

```python
from std_msgs.msg import String # Example message type

# Inside your node's __init__ method
self.subscription = self.create_subscription(
    String,
    'chatter',
    self.listener_callback, # Function to call when a message is received
    10) # QoS depth
self.subscription  # prevent unused variable warning

def listener_callback(self, msg):
    self.get_logger().info('I heard: "%s"' % msg.data)
```
The `listener_callback` function is crucial. It takes the received message as an argument, allowing the subscriber node to process the incoming data. This event-driven approach means the subscriber only wakes up and performs work when new data is available on the subscribed topic.

## 2.3 Example: Simple Sensor Data Exchange Simulation

This section will demonstrate a practical example of a publisher and subscriber working together. We will create a simple "talker" node that publishes a counter value (simulating sensor data) and a "listener" node that subscribes to this data and prints it to the console.

### Running the Example

To run the `simple_publisher.py` and `simple_subscriber.py` example, you will first need to build your ROS 2 workspace. Navigate to the root of your `module1-ros2-humanoid-control` directory and execute:

```bash
colcon build --symlink-install
```

After a successful build, source your workspace to make the new packages available:

```bash
source install/setup.bash
```

Now, you can run the publisher and subscriber nodes using the launch file we created:

```bash
ros2 launch module1_ros2_humanoid_control simple_comm_launch.py
```

This command will start both the `simple_publisher_node` and `simple_subscriber_node` simultaneously. You should see output from both nodes in your terminal, with the publisher sending integer data and the subscriber receiving and printing it.

Alternatively, you can run them individually in separate terminals:

**Terminal 1 (Publisher):**
```bash
ros2 run module1_ros2_humanoid_control simple_publisher
```

**Terminal 2 (Subscriber):**
```bash
ros2 run module1_ros2_humanoid_control simple_subscriber
```

## 2.4 Best Practices for Topic Communication

Effective topic communication involves more than just sending and receiving messages. Adhering to best practices ensures your ROS 2 applications are robust, efficient, and debuggable.

### Choosing Appropriate Message Types

Always select the most specific and appropriate message type for the data you are transmitting. ROS 2 provides a rich set of standard message types (e.g., `std_msgs`, `sensor_msgs`, `geometry_msgs`). Using these standard types improves interoperability and readability. If no standard message type fits your needs, you can define custom message types.

### Understanding QoS Settings for Topics

Quality of Service (QoS) policies are critical for fine-tuning communication to meet specific application requirements. For topics, key QoS settings include:
-   **Reliability**: `RELIABLE` (guarantees delivery, retries on failure) versus `BEST_EFFORT` (no retransmissions, fastest delivery). Choose `RELIABLE` for critical data (e.g., commands) and `BEST_EFFORT` for non-critical, high-frequency data (e.g., camera feeds where dropping a frame is acceptable).
-   **Durability**: `TRANSIENT_LOCAL` (new subscribers receive the last message published) versus `VOLATILE` (new subscribers only receive messages published after they subscribe). `TRANSIENT_LOCAL` is useful for transmitting static configuration or initial states.
-   **History**: `KEEP_LAST` (stores a specified number of messages) or `KEEP_ALL` (stores all messages up to resource limits).

Carefully choosing QoS profiles can significantly impact the performance and behavior of your ROS 2 system.

### Debugging Topic Communication

When issues arise with topic communication, several tools can help diagnose the problem:
-   `ros2 topic list`: Lists all active topics.
-   `ros2 topic info <topic_name>`: Shows publisher/subscriber count and message type.
-   `ros2 topic echo <topic_name>`: Displays messages being published on a topic in real-time.
-   `ros2 topic hz <topic_name>`: Reports the publishing rate of a topic.
-   `rqt_graph`: Provides a graphical representation of the ROS 2 computation graph, showing nodes and their connections.

These tools are invaluable for verifying that messages are being published and subscribed correctly, and for understanding the flow of data within your ROS 2 system.
