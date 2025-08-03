import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class CameraRelay(Node):
    def __init__(self):
        super().__init__('camera_relay')

        # Subscriber
        self.subscription = self.create_subscription(
            Image,
            'rgb', # Subscribe to topic
            self.image_callback,
            10 # Queue
        )
        
        # Publisher
        self.publisher = self.create_publisher(
            Image,
            'image_processed', # Publish to topic
            10 # Queue
        )

    def image_callback(self, msg: Image):

        # Republishing the received image
        self.publisher.publish(msg)
        self.get_logger().info('Published image')
        

def main(args=None):
    rclpy.init(args=args)
    node = CameraRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
