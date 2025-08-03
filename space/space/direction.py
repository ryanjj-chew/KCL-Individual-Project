import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
from sensor_msgs.msg import CameraInfo
import numpy as np
from collections import deque

class Direction(Node):
    def __init__(self):
        super().__init__('direction_subscriber')
        self.depth = None
        
        # Camera intrinsics
        self.K = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.tolerance = 300

        self.net_speed = 2.0 # m/s
        self.target_vectors = deque(maxlen=20) # vector from camera to object
        self.object_vector = deque(maxlen=19) # vector of the object's movement
        self.last_vector = None
        self.previous_center = None

        # Camera translation from wrist center
        
        
        self.intrinsics_subscriber = self.create_subscription(
            CameraInfo,
            'camera_info',
            self.intrinsics_callback,
            10
        )

        # Subscription for depth
        self.depth_subscription = self.create_subscription(
            Float64,
            'depth_value',
            self.depth_callback,
            10
        )

        # Subscription for receiving bounding box coordinates
        self.bbox_subcription = self.create_subscription(
            Float64MultiArray,
            'bounding_box', # Topic of bounding box coordinates
            self.bbox_callback,
            10
        )

        # Publisher for direction vector
        self.direction_publisher = self.create_publisher(
            Vector3,
            'direction', # Publishes the vector FROM the CAMERA VIEWPORT
            10
        )

    def intrinsics_callback(self, msg: CameraInfo):
        # Camera intrinsic matrix K (3x3)
        #self.K = np.array(msg.k).reshape(3,3)
        #self.get_logger().info(f'Camera intrinsic matrix K: \n{self.K}')
        
        # K = [fx 0 cx, 0 fy cy, 0 0 1]
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def depth_callback(self, msg: Float64):
        self.depth = msg.data
        self.get_logger().info(f"Depth received at {self.depth} m")

    def bbox_callback(self, msg: Float64MultiArray):
        if self.depth is None:
            return
        
        data = msg.data
        center_x, center_y, *_ = data
        self.get_logger().info(f"Center: ({center_x}, {center_y})")

        if self.previous_center is not None: # Check for new object
            dx = abs(center_x - self.previous_center[0])
            dy = abs(center_y - self.previous_center[1])

            if dx > self.tolerance or dy > self.tolerance: 
                self.target_vectors.clear()
                self.object_vector.clear()
                self.last_vector = None
                self.avg_target_vector = None
                self.avg_obj_vector = None

        # Obtain vector from x,y,z coordinates
        # Formula = (u-c)*(Z/f)
        vector_x = (center_x - self.cx) * self.depth / self.fx
        vector_y = (center_y - self.cy) * self.depth / self.fy
        vector_z = self.depth
        v = np.array([vector_x, vector_y, vector_z]) # latest vector

        if self.last_vector is not None:
            self.object_vector.append(v - self.last_vector)
        self.target_vectors.append(v)
        self.last_vector = v
        self.previous_center = (center_x, center_y)

        if len(self.target_vectors) == self.target_vectors.maxlen: # proceed at 20 samples
            #avg_target_vector = np.mean(self.target_vectors, axis=0)
            latest_target_vector = self.target_vectors[-1]
            avg_obj_vector = np.mean(self.object_vector, axis=0) if self.object_vector else np.zeros(3)
        
            distance = np.linalg.norm(latest_target_vector)
            time = distance / self.net_speed
            offset_vector = latest_target_vector + avg_obj_vector * time # double check if avg vector or latest vector

            msg = Vector3()
            msg.x = float(offset_vector[0])
            msg.y = float(offset_vector[1])
            msg.z = float(offset_vector[2])
            self.direction_publisher.publish(msg)
            self.get_logger().info(f"Offset vector: {msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f}")

        


def main(args=None):
    rclpy.init(args=args)
    node = Direction()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
