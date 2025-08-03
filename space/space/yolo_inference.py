import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import ultralytics
from ultralytics import YOLO
import numpy as np
import torch

class YoloInference(Node):
    def __init__(self):
        super().__init__('yolo_inference')

        self.model = YOLO("/home/ryan/ros2_ws/src/space/space/yolov11n.pt")
        self.bridge = CvBridge()

        # Subscriber
        self.subscription = self.create_subscription(
            Image,
            'rgb',
            self.yolo_callback,
            10
        )
        
        # Publisher
        self.publisher = self.create_publisher(
            Float64MultiArray,
            'bounding_box',
            10
        )
        
        # Bounding box publisher
        self.bbox_publisher = self.create_publisher(
            Image,
            'bbox_image',
            10
        )
        	

    def yolo_callback(self, msg: Image):
        # Convert image message type to opencv image type
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
            bgr_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            results = self.model(bgr_image)

            # YOLO model bounding box center coordinates in pixels and class for image
            for box in results[0].boxes:
                xmin, ymin, xmax, ymax = box.xyxy.tolist()[0]

                # Calculate center pixel
                center_x = (xmin + xmax) / 2.0
                center_y = (ymin + ymax) / 2.0
                
                # ---- DRAW the rectangle and class label on bgr_image ----
                cv2.rectangle(
                    bgr_image,
                    (int(xmin), int(ymin)),
                    (int(xmax), int(ymax)),
                    (0, 255, 0), 2
                )
                cv2.putText(
                    bgr_image,
                    str(int(box.cls)),
                    (int(xmin), int(ymin) - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0), 2
                )

                # convert BGR back→ROS Image in rgb8
                annotated = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
                out_msg = self.bridge.cv2_to_imgmsg(annotated, 'rgb8')
                out_msg.header = msg.header

                self.bbox_publisher.publish(out_msg)

                # Publish
                bounding_box = Float64MultiArray()
                bounding_box.data = [center_x, center_y, xmin, ymin, xmax, ymax]

                self.publisher.publish(bounding_box)
                self.get_logger().info(f"Detected center pixel: x={center_x:.1f}, y={center_y:.1f}")
        
        except Exception as e:
            self.get_logger().info(f"YOLO inference failed: {e}")
            pass

def main(args=None):
    rclpy.init(args=args)
    node = YoloInference()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
