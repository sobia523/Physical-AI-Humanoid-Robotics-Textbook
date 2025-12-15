import rclpy
from rclpy.node import Node
from custom_interfaces.msg import ObjectDetections, SimulatedObject
from geometry_msgs.msg import Pose
from std_msgs.msg import Header

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.publisher_ = self.create_publisher(ObjectDetections, 'detected_objects', 10)
        self.timer = self.create_timer(2.0, self.publish_dummy_detections) # Publish dummy detections every 2 seconds
        self.get_logger().info('Perception Node started. Publishing to /detected_objects.')

    def publish_dummy_detections(self):
        msg = ObjectDetections()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"

        # Dummy red cube
        red_cube = SimulatedObject()
        red_cube.id = "red_cube_001"
        red_cube.type = "cube"
        red_cube.pose = Pose()
        red_cube.pose.position.x = 0.5
        red_cube.pose.position.y = -0.5
        red_cube.pose.position.z = 0.025
        red_cube.pose.orientation.w = 1.0 # default orientation
        # Add other properties if needed
        # red_cube.properties = {"color": "red"}

        msg.objects.append(red_cube)

        self.publisher_.publish(msg)
        self.get_logger().info('Published dummy object detections.')

def main(args=None):
    rclpy.init(args=args)
    perception_node = PerceptionNode()
    rclpy.spin(perception_node)
    perception_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
