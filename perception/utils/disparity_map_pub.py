#!/usr/bin/env python3

from __future__ import print_function

import numpy as np
import cv2
import time
import rclpy
from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from std_msgs.msg import Header
from message_filters import Subscriber, ApproximateTimeSynchronizer
from fs_msgs.msg import TrackStampedWithCovariance, TrackStamped
from yolov8_msgs.msg import Yolov8Inference
from amp_perception.perception.utils.perception_calc import PerceptionProcess

bridge = CvBridge()


class Disparity_Publisher(Node):

    def __init__(self):
        super().__init__('disparity_map_pub')

        ## colocar como parametro posteriormente
        left_config_file_name = "OAKDLR_left_22_04.yaml"
        right_config_file_name = "OAKDLR_right_22_04.yaml"

        baseline = 0.15

        self.calc = PerceptionProcess(baseline, left_config_file_name, right_config_file_name)

        self.img_left = Subscriber(self, Image, "camera/left")
        self.img_right = Subscriber(self, Image, "camera/right")

        self.disp_patinho_map = self.create_publisher(Image, "disparity", 10)
        #self.img_L_rect = self.create_publisher(Image, "image_rect/left", 10)
        #self.img_R_rect = self.create_publisher(Image, "image_rect/right", 10)
        #self.combinated = self.create_publisher(Image, "combinated", 10)

        self.periodo = 0
        self.total = 0
        
        max_delay = 0.1
        self.time_sync = ApproximateTimeSynchronizer([self.img_left,self.img_right],10,max_delay)
        self.time_sync.registerCallback(self.sync_callback)
     

    def sync_callback(self, img_L, img_R):
        start_time = time.time()
        img_L_rect_msg, img_R_rect_msg = self.calc.approximate_stereo_rectify(img_L, img_R)
        self.get_logger().warn("chegou aqui")
        disp_map = self.calc.DisparityProcess(img_L_rect_msg, img_R_rect_msg)[1]

        #combinated = self.draw_epilines(img_L_rect_msg, img_R_rect_msg)
        #combinated = bridge.cv2_to_imgmsg(combinated)
        #self.combinated.publish(combinated)

        disp_map = bridge.cv2_to_imgmsg(disp_map)
        disp_map.header = img_L.header

        img_L_rect_msg.header = img_L.header
        img_R_rect_msg.header = img_R.header
        
        self.disp_patinho_map.publish(disp_map)

        #self.img_L_rect.publish(img_L_rect_msg)
        #self.img_R_rect.publish(img_R_rect_msg)
            
        end_time = time.time()
        self.total += end_time - start_time
        self.periodo += 1
        periodo_medio = (self.total/self.periodo)

        self.get_logger().info(f"Frequencia media do Callback: {1/periodo_medio:.4f} hz ")

    def draw_epilines(self, imgL_rect, imgR_rect):
        n_lines=20
        imgL_rect = bridge.imgmsg_to_cv2(imgL_rect)
        imgR_rect = bridge.imgmsg_to_cv2(imgR_rect)

        h, w = imgL_rect.shape[:2]
        combined = np.hstack([imgL_rect, imgR_rect])
        
        for y in range(0, h, h // n_lines):
            cv2.line(combined, (0, y), (w * 2, y), (0, 255, 0), 1)

        return combined


def main(args=None):
    rclpy.init(args=args)
    
    disparity_map_pub = Disparity_Publisher()
    
    rclpy.spin(disparity_map_pub)
    disparity_map_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    cv2.destroyAllWindows()
    