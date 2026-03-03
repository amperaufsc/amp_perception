#!/usr/bin/env python3

from __future__ import print_function

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import Image, CameraInfo
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge
from std_msgs.msg import Header
from message_filters import Subscriber, ApproximateTimeSynchronizer
from fs_msgs.msg import TrackStampedWithCovariance, TrackStamped
from yolov8_msgs.msg import Yolov8Inference
from perception_calc import PerceptionProcess
import yaml
import time
import threading

bridge = CvBridge()


class NoDisparidadeShowWindow(Node):

    def __init__(self):
        super().__init__('disp_map_show')
        self.get_logger().info("DisparityImage foi iniciallizado")
        baseline = 0.15
        self.calc = PerceptionProcess(baseline)
                
        #self.disp_base_map = Subscriber(self, Image, "/disparity_map/patinho")
        #self.disp_filtered_map = Subscriber(self, Image, "/oak/stereo/image_raw")
        #self.yolo_sub = Subscriber(self, Yolov8Inference, "/inferenceresult")
        self.img_L = Subscriber(self, Image, "/oak/left/image_raw")
        self.img_R = Subscriber(self, Image, "/oak/right/image_raw")

        self.disp_patinho_map = self.create_publisher(Image, "/disparity_map/patinho", 10)
        
        max_delay = 0.05
        self.time_sync = ApproximateTimeSynchronizer([self.img_L, self.img_R],10,max_delay)
        self.time_sync.registerCallback(self.sync_callback)
        
        #Criação de uma thread dedicada a atualizar GUI, para que a atualização aconteça paralelamente ao processamento das imagens e, assim, não uma função "independe" da outra
        
        self.latest_disparity_map = None
        self.lock = threading.Lock()
        self.gui_thread = threading.Thread(target=self.display_loop)
        self.gui_thread.start()
        
        self.get_logger().info("init finalizado")
        

    def sync_callback(self, img_L_raw, img_R_raw):

        img_L_rect, img_R_rect, kL, pL = self.calc.approximate_stereo_rectify(img_L_raw, img_R_raw)
        base_map = self.calc.DisparityProcess(img_L_rect, img_R_rect)[1]

        img_L_rect = bridge.imgmsg_to_cv2(img_L_rect)
        img_R_rect = bridge.imgmsg_to_cv2(img_R_rect)

        with self.lock:
            self.latest_disparity_map = base_map.copy()

        base_map = bridge.cv2_to_imgmsg(base_map)
        base_map.header = img_L_raw.header
        self.disp_patinho_map.publish(base_map)

    def display_loop(self):
        cv2.namedWindow("Mapa de Disparidade V0", cv2.WINDOW_NORMAL)

        while rclpy.ok():
            with self.lock:
                display_image = self.latest_disparity_map
            if display_image is not None:
            
                cv2.imshow("imgL", display_image)
                
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break
            time.sleep(0.01)
        cv2.destroyAllWindows()

    def draw_yolo_boxes(self, image, detections):
        yolo = detections.yolov8_inference
        img_out = image.copy()
        color=(60, 20, 220)
        
        for det in yolo:
            x1 = det.top
            y1 = det.left
            x2 = det.bottom
            y2 = det.right

            cv2.rectangle(img_out, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
        return img_out

def main(args=None):
    rclpy.init(args=args)
    
    disp_map_show = NoDisparidadeShowWindow()
    
    rclpy.spin(disp_map_show)
    disp_map_show.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    cv2.destroyAllWindows()
    