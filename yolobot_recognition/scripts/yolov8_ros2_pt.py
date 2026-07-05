#!/usr/bin/env python3

from ultralytics import YOLO
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from yolov8_msgs.msg import InferenceWithConfidence
from yolov8_msgs.msg import Yolov8Inference

bridge = CvBridge()

class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        self.yolov8_inference = Yolov8Inference()

        self.subscription = self.create_subscription(
            Image,
            'image',
            self.camera_callback,  
            10)
        self.subscription 

        self.yolov8_pub = self.create_publisher(Yolov8Inference, "inferenceresult", 1)
        self.img_pub = self.create_publisher(Image, "inferenceimg", 1)

        # self.declare_parameter('yolov8_path', 'src/as_amp/yolobot_recognition/scripts/best.pt')
        self.declare_parameter('yolov8_path', 'src/amp_perception/yolobot_recognition/scripts/best.pt')

        self.yolov8_path = str(self.get_parameter('yolov8_path').value)

        # TODO: set confidence parameter to be from launch or (exclusive) from the present script
        self.declare_parameter('confidence_threshold',0.5)
        self.confidence_threshold = self.get_parameter('confidence_threshold').value

        self.get_logger().info(f"{self.confidence_threshold}")

        self.model = YOLO(self.yolov8_path)

    def camera_callback(self, data):

        confidence = self.confidence_threshold

        img = bridge.imgmsg_to_cv2(data, "bgr8")
        '''
        img_resized = cv2.resize(img, (640, 640)) 

        img_normalized = img_resized / 255.0  

        results = self.model(img_normalized)
        '''
        results = self.model(img, conf=confidence, device=0)

        self.yolov8_inference.header = data.header

        for r in results:
            boxes = r.boxes
            for box in boxes:
                self.inference_result = InferenceWithConfidence()
                b = box.xyxy[0].to('cpu').detach().numpy().copy()  # get box coordinates in (top, left, bottom, right) format
                c = box.cls
                self.inference_result.class_name = self.model.names[int(c)]
                self.inference_result.top = int(b[0])
                self.inference_result.left = int(b[1])
                self.inference_result.bottom = int(b[2])
                self.inference_result.right = int(b[3])
                self.inference_result.confidence = float(box.conf)
                self.yolov8_inference.yolov8_inference.append(self.inference_result) # type: ignore

            #camera_subscriber.get_logger().info(f"{self.yolov8_inference}")

        annotated_frame = results[0].plot()
        img_msg = bridge.cv2_to_imgmsg(annotated_frame)  

        self.img_pub.publish(img_msg)
        self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.yolov8_inference.clear() # type: ignore

if __name__ == '__main__':
    rclpy.init(args=None)
    camera_subscriber = Camera_subscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()

