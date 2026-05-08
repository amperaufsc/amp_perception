#!/usr/bin/env python3

from __future__ import print_function

import numpy as np
import cv2
import os

import rclpy
from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import Image, CameraInfo, RegionOfInterest
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from perception_calc import PerceptionProcess
import yaml
import time

bridge = CvBridge()


class Camera_Info_Pub(Node):

    def __init__(self):
        super().__init__('Camera_Info')

        self.perception_calc = PerceptionProcess(0.15)
                
        self.img_L = Subscriber(self, Image, "/oak/left/image_raw")
        self.img_R = Subscriber(self, Image, "/oak/right/image_raw")

        self.camera_info_left = self.create_publisher(CameraInfo, "/patinho/left/camera_info", 10)
        self.camera_info_right = self.create_publisher(CameraInfo, "/patinho/right/camera_info", 10)

        self.img_l_pub = self.create_publisher(Image, "/patinho/left/image_raw", 10)
        self.img_r_pub = self.create_publisher(Image, "/patinho/right/image_raw", 10)

        max_delay = 0.05
        self.time_sync = ApproximateTimeSynchronizer([self.img_L, self.img_R],10,max_delay)
        self.time_sync.registerCallback(self.sync_callback)
        
        self.get_logger().info("init finalizado")
        

    def sync_callback(self, img_L_raw, img_R_raw):
        [k_left, d_left, r_left, p_left] = self.perception_calc.camera_matrix[0]
        [k_right, d_right, r_right, p_right] = self.perception_calc.camera_matrix[1]

        cameraInfoLeft = self.create_camera_info(768, 480, k_left, d_left, r_left, p_left, img_L_raw.header)
        cameraInfoRight = self.create_camera_info(768, 480, k_right, d_right, r_right, p_right, img_R_raw.header)

        self.camera_info_left.publish(cameraInfoLeft)
        self.camera_info_right.publish(cameraInfoRight)

        self.img_l_pub.publish(img_L_raw)
        self.img_r_pub.publish(img_R_raw)

        self.get_logger().warn("Publicou")

    def create_camera_info(self, width, height, k_matrix, d_coeffs, r_matrix, p_matrix, header):
        info = CameraInfo()
        info.header = header
        # info.header.stamp = node.get_clock().now().to_msg() # Opcional: adicionar timestamp aqui
        
        info.width = width
        info.height = height
        
        # Matriz Intrínseca K (flatten para lista de 9 elementos)
        info.k = [float(x) for x in k_matrix.flatten()]
        
        # Coeficientes de Distorção D
        info.d = [float(x) for x in d_coeffs.flatten()]
        
        # Matriz de Retificação R (flatten para lista de 9 elementos)
        info.r = [float(x) for x in r_matrix.flatten()]
        
        # Matriz de Projeção P (flatten para lista de 12 elementos)
        info.p = [float(x) for x in p_matrix.flatten()]
        
        # Modelo de distorção padrão
        info.distortion_model = "plumb_bob"

        info.roi = self.create_roi(0, 0, 0, 0, rectify=False)
        
        return info

    def create_roi(self, x, y, h, w, rectify):

        roi = RegionOfInterest()
        roi.x_offset = int(x)
        roi.y_offset = int(y)
        roi.height = int(h)
        roi.width = int(w)
        roi.do_rectify = rectify
        
        return roi
            

def main(args=None):
    rclpy.init(args=args)
    
    Camera_Info = Camera_Info_Pub()
    
    rclpy.spin(Camera_Info)
    Camera_Info.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    cv2.destroyAllWindows()
    