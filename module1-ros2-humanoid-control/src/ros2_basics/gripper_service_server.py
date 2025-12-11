import rclpy
from rclpy.node import Node
from module1_ros2_humanoid_control.srv import SetGripperState

class GripperServiceServer(Node):

    def __init__(self):
        super().__init__('gripper_service_server')
        self.srv = self.create_service(SetGripperState, 'set_gripper_state', self.set_gripper_state_callback)
        self.gripper_open = False # Initial state
        self.get_logger().info('Gripper service server started.')

    def set_gripper_state_callback(self, request, response):
        if request.open_gripper:
            self.gripper_open = True
            response.success = True
            self.get_logger().info('Gripper opened.')
        else:
            self.gripper_open = False
            response.success = True
            self.get_logger().info('Gripper closed.')
        return response

def main(args=None):
    rclpy.init(args=args)
    gripper_service_server = GripperServiceServer()
    rclpy.spin(gripper_service_server)
    gripper_service_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
