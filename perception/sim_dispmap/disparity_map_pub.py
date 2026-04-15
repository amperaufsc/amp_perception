#!/usr/bin/env python3

from __future__ import print_function

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from std_msgs.msg import Header
from message_filters import Subscriber, ApproximateTimeSynchronizer
from fs_msgs.msg import TrackStampedWithCovariance, TrackStamped
from yolov8_msgs.msg import Yolov8Inference
from perception_calc import PerceptionProcess

bridge = CvBridge()


class Disparity_Publisher(Node):

    def __init__(self):
        super().__init__('disparity_map_pub')
        self.baseline = 0.15
        self.calc = PerceptionProcess(self.baseline)

        self.img_left = Subscriber(self, Image, "/oak/left/image_raw")
        self.img_right = Subscriber(self, Image, "/oak/right/image_raw")

        self.disp_patinho_map = self.create_publisher(Image, "/disparity_map/teste", 10)
        self.img_L_rect = self.create_publisher(Image, "/image_rect/left", 10)
        self.img_R_rect = self.create_publisher(Image, "/image_rect/right", 10)
        self.img_lines = self.create_publisher(Image, "/image_rect/lines", 10)
        
        max_delay = 0.5
        self.time_sync = ApproximateTimeSynchronizer([self.img_left,self.img_right],10,max_delay)
        self.time_sync.registerCallback(self.sync_callback)        

    def sync_callback(self, img_L, img_R):
        img_L_rect_msg, img_R_rect_msg = self.calc.approximate_stereo_rectify(img_L, img_R)
        self.get_logger().warn("chegou aqui")
        disp_map = self.calc.DisparityProcess(img_L_rect_msg, img_R_rect_msg)[1]

        combined_img = self.draw_epilines(img_L_rect_msg, img_R_rect_msg)

        combined_img = bridge.cv2_to_imgmsg(combined_img)
        disp_map = bridge.cv2_to_imgmsg(disp_map)
        disp_map.header = img_L_rect_msg.header
        
        self.disp_patinho_map.publish(disp_map)
        self.img_lines.publish(combined_img)
        self.img_L_rect.publish(img_L_rect_msg)
        self.img_R_rect.publish(img_R_rect_msg)
            
        self.get_logger().info("transformou")

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
    