import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor # Import for spin_until_future_complete
from module1_ros2_humanoid_control.srv import SetGripperState
import sys

class GripperServiceClient(Node):

    def __init__(self):
        super().__init__('gripper_service_client')
        self.cli = self.create_client(SetGripperState, 'set_gripper_state')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetGripperState.Request()
        self.get_logger().info('Gripper service client ready.')

    def send_request(self, open_gripper_state):
        self.req.open_gripper = open_gripper_state
        self.future = self.cli.call_async(self.req)
        # Use a SingleThreadedExecutor to spin until the future is complete
        # This is a common pattern for clients to wait for a service response
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        executor.spin_until_future_complete(self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    gripper_service_client = GripperServiceClient()

    if len(sys.argv) != 2:
        gripper_service_client.get_logger().info('Usage: ros2 run module1_ros2_humanoid_control gripper_service_client <true/false>')
        return

    open_gripper = sys.argv[1].lower() == 'true'
    
    response = gripper_service_client.send_request(open_gripper)
    if response.success:
        gripper_service_client.get_logger().info(f'Service call successful. Gripper state set to {"open" if open_gripper else "closed"}.')
    else:
        gripper_service_client.get_logger().info('Service call failed.')
    
    gripper_service_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
