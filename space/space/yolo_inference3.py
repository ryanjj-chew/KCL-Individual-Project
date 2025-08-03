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

    def yolo_callback(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        #bgr_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        results = self.model(cv_image)
        for result in results:
            print(result.boxes.xyxy)


def main(args=None):
    rclpy.init(args=args)
    node = YoloInference()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()