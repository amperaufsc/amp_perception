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

        self.img_left = Subscriber(self, Image, "/oak/left/image_rect")
        self.img_right = Subscriber(self, Image, "/oak/right/image_rect")

        self.disp_patinho_map = self.create_publisher(Image, "/disparity_map/patinho", 10)
        
        max_delay = 0.05
        self.time_sync = ApproximateTimeSynchronizer([self.img_left,self.img_right],10,max_delay)
        self.time_sync.registerCallback(self.sync_callback)        

    def sync_callback(self, img_L, img_R):
        try:
            disp_map = self.calc.DisparityProcess(img_L, img_R)[1]
            disp_map = bridge.cv2_to_imgmsg(disp_map)
            disp_map.header = img_L.header
            self.disp_patinho_map.publish(disp_map)
            self.get_logger().warn("Publicando disparidade")
        except:
            self.get_logger().warn("Pulou quadros papito")
            
def main(args=None):
    rclpy.init(args=args)
    
    disparity_map_pub = Disparity_Publisher()
    
    rclpy.spin(disparity_map_pub)
    disparity_map_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    cv2.destroyAllWindows()
    