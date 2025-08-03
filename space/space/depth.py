import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import CameraInfo
import numpy as np
import cv2
from cv_bridge import CvBridge
import math

class Depth(Node):
    def __init__(self):
        super().__init__('depth_subscriber')
        self.bridge = CvBridge()

        self.center_x = None
        self.center_y = None
        self.width = None
        self.height = None
        self.K = None
        self.D = None
        self.P = None

        # Camera translation from wrist center
        self.trans_x = 0
        self.trans_y = 0
        self.trans_z = 0

        # Camera Intrinsics
        self.intrinsics_subscriber = self.create_subscription(
            CameraInfo,
            'camera_info',
            self.intrinsics_callback,
            10
        )

        self.center_subscriber = self.create_subscription(
            Float64MultiArray,
            'bounding_box',
            self.center_callback,
            10
        )

        self.depth_subscriber = self.create_subscription(
            Image,
            'depth',
            self.depth_callback,
            10
        )

        self.depth_publisher = self.create_publisher(
            Float64,
            'depth_value',
            10
        )

    def center_callback(self, msg: Float64MultiArray):
        self.center_x = int(msg.data[0])
        self.center_y = int(msg.data[1])
        self.get_logger().info(f'Received center pixel: ({self.center_x}, {self.center_y})')

    def intrinsics_callback(self, msg: CameraInfo):
        # Camera intrinsic matrix K (3x3)
        self.K = np.array(msg.k).reshape(3,3)
        #self.get_logger().info(f'Camera intrinsic matrix K: \n{self.K}')

        # Camera distortion matrix D (5x1)
        self.D = np.array(msg.d)
        #self.get_logger().info(f'Camera distortion matrix D: \n{self.D}')

        # Camera projection P (3x4)
        self.P = np.array(msg.p)
        #self.get_logger().info(f'Camera intrinsic matrix P: \n{self.P}')

        # Camera image resolution pixels
        self.width = msg.width
        self.height = msg.height
        #self.get_logger().info(f'Camera image size: {self.width} x {self.height}')

    def depth_callback(self, msg: Image):
        if self.center_x is None or self.center_y is None:
            self.get_logger().warn("Center pixel not received")
            return
        
        if self.width is None or self.height is None:
            self.get_logger().warn("Camera info not received")
            return

        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        except Exception as e:
            self.get_logger().error(f'Depth image conversion failed: {e}')

        # Clamp and index
        u = int(min(max(self.center_x, 0), self.width  - 1))
        v = int(min(max(self.center_y, 0), self.height - 1))

        # Obtain depth
        depth = self.depth_image[v,u]
        if math.isinf(depth) == True or depth > 50.0:
            return

        # Depth in meters
        depth_msg = Float64()
        depth_msg.data = depth / 1.0
        self.depth_publisher.publish(depth_msg)
        
        self.get_logger().info(f'Depth at point ({u}, {v}), = {depth:.3f} m')
            
def main(args=None):
    rclpy.init(args=args)
    node = Depth()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
