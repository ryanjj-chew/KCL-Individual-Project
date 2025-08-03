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
        self.cam_msg = None
        self.depth_msg = None
        self.bbox_msg = None
        
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

        self.create_subscription(CameraInfo, 'camera_info', self.cam_callback, 10)
        self.create_subscription(Float64, 'depth_value', self.depth_callback, 10)
        self.create_subscription(Float64MultiArray, 'bounding_box', self.bbox_callback, 10)

        # Publishes the vector FROM the CAMERA VIEWPORT
        self.direction_publisher = self.create_publisher(Vector3, 'direction', 10)


    def cam_callback(self, msg):
        self.cam_msg = msg
        self.try_sync()

    def depth_callback(self, msg):
        self.depth_msg = msg
        self.try_sync()

    def bbox_callback(self, msg):
        self.bbox_msg = msg
        self.try_sync()   

    def try_sync(self):
        if self.cam_msg and self.depth_msg and self.bbox_msg:
            self.synced_callback(self.cam_msg, self.depth_msg, self.bbox_msg)

    def synced_callback(self, cam_info: CameraInfo, depth_msg: Float64, bbox_msg: Float64MultiArray):
        # K = [fx 0 cx, 0 fy cy, 0 0 1]
        self.fx = cam_info.k[0]
        self.fy = cam_info.k[4]
        self.cx = cam_info.k[2]
        self.cy = cam_info.k[5]

        depth = depth_msg.data
        self.get_logger().info(f"Depth received at {depth} m")
        
        center_x, center_y, *_ = bbox_msg.data
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
        vector_x = (center_x - self.cx) * depth / self.fx
        vector_y = (center_y - self.cy) * depth / self.fy
        vector_z = depth
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
            self.cam_msg = self.depth_msg = self.bbox_msg = None


def main(args=None):
    rclpy.init(args=args)
    node = Direction()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
