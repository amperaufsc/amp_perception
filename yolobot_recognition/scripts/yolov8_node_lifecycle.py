#!/usr/bin/env python3

from ultralytics import YOLO
import cv2
import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from yolov8_msgs.msg import InferenceWithConfidence
from yolov8_msgs.msg import Yolov8Inference

bridge = CvBridge()

class LifecycleCameraSubscriber(LifecycleNode):

    def __init__(self):
        super().__init__('yolo_node')
        
        self.yolov8_inference = Yolov8Inference()
        self.subscription = None
        self.yolov8_pub = None
        self.img_pub = None
        self.model = None

        self.declare_parameter('yolov8_path', '/src/amp_perception/yolobot_recognition/scripts/teste_19_05/best.pt')
        self.declare_parameter('confidence_threshold', 0.5)

        self.get_logger().info("Nó de Inferência YOLOv8 Inicializado (Unconfigured)")

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        try:
            self.get_logger().info("Configurando o nó (Carregando Pesos do YOLO)...")
            
            self.yolov8_path = self.get_parameter('yolov8_path').value
            self.confidence_threshold = self.get_parameter('confidence_threshold').value
            
            self.get_logger().info(f"Threshold: {self.confidence_threshold}")

            self.model = YOLO(self.yolov8_path)

            self.yolov8_pub = self.create_lifecycle_publisher(Yolov8Inference, "inferenceresult", 1)
            self.img_pub = self.create_lifecycle_publisher(Image, "inferenceimg", 1)

            self.subscription = self.create_subscription(
                Image,
                'image',
                self.camera_callback,  
                10)

            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Erro na configuração: {e}")
            return TransitionCallbackReturn.ERROR

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Ativando a Inferência YOLOv8...")
        super().on_activate(state)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Desativando a Inferência YOLOv8...")
        super().on_deactivate(state)
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Limpando a memória do nó...")

        self.destroy_subscription(self.subscription)
        self.destroy_lifecycle_publisher(self.yolov8_pub)
        self.destroy_lifecycle_publisher(self.img_pub)
        
        self.subscription = None
        self.yolov8_pub = None
        self.img_pub = None
        self.model = None
        
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Encerrando o nó com segurança...")
        self.destroy_subscription(self.subscription)
        self.destroy_lifecycle_publisher(self.yolov8_pub)
        self.destroy_lifecycle_publisher(self.img_pub)
        return TransitionCallbackReturn.SUCCESS

    def camera_callback(self, data):
        if self.img_pub is None or not self.img_pub.is_activated:
            return

        confidence = self.confidence_threshold

        img = bridge.imgmsg_to_cv2(data, "bgr8")

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
                self.yolov8_inference.yolov8_inference.append(self.inference_result)

            #self.get_logger().info(f"{self.yolov8_inference}")

        annotated_frame = results[0].plot()
        img_msg = bridge.cv2_to_imgmsg(annotated_frame)  

        self.img_pub.publish(img_msg)
        self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.yolov8_inference.clear()

def main(args=None):
    rclpy.init(args=args)
    
    node = LifecycleCameraSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()