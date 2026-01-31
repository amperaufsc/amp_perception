#!/usr/bin/env python3
from __future__ import print_function

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge
from std_msgs.msg import Header
from message_filters import Subscriber, ApproximateTimeSynchronizer
from fs_msgs.msg import TrackStampedWithCovariance
from perception_calc import PerceptionProcess
from yolov8_msgs.msg import Yolov8Inference

bridge = CvBridge()

class Cone_Track_Process(Node):

    def __init__(self):
        super().__init__('cone_track_node')
        self.get_logger().info("Nó foi iniciallizado")
        endereco_matriz_intrinsica_left = '/home/otaviogoulart/ws/src/amp_perception/perception/config/fsds_left.yaml'
        endereco_matriz_intrinsica_right = '/home/otaviogoulart/ws/src/amp_perception/perception/config/fsds_right.yaml'
        baseline = 0.32
        
        self.calc = PerceptionProcess(endereco_matriz_intrinsica_left, endereco_matriz_intrinsica_right, baseline)
                
        self.image_left_sub = Subscriber(self, Image, "image/left")
        self.image_right_sub = Subscriber(self, Image, "image/right")
        self.yolo_inf_sub = Subscriber(self, Yolov8Inference, "inferenceresult")
        
        self.base_disp_map = self.create_publisher(Image, "/disparity_map/base", 10)
        self.filtered_disp_map = self.create_publisher(Image, "/disparity_map/filtered", 10)
        self.Track_Stamped_Base_Pub = self.create_publisher(TrackStampedWithCovariance, "/track_pub/base",10)
        self.Track_Stamped_Filtered_Pub = self.create_publisher(TrackStampedWithCovariance, "/track_pub/filtered",10)
        
        max_delay = 0.1
        self.time_sync = ApproximateTimeSynchronizer([self.image_left_sub,self.image_right_sub,self.yolo_inf_sub],10,max_delay)
        self.time_sync.registerCallback(self.sync_callback)
        
        self.get_logger().info("init finalizado")
        

    def sync_callback(self, imgL_raw_ros_msg, imgR_raw_ros_msg, yoloinference):
        disparity_maps = self.calc.DisparityProcess(imgL_raw_ros_msg, imgR_raw_ros_msg)
        base_map = disparity_maps[0]
        filtered_map = disparity_maps[1]
        track_base_map = self.calc.object_on_map(yoloinference, base_map, imgL_raw_ros_msg, imgR_raw_ros_msg)[0]
        track_filtered_map = self.calc.object_on_map(yoloinference, filtered_map, imgL_raw_ros_msg, imgR_raw_ros_msg)[0]
        
        track_stamped_base = self.Track_Stamped_With_Covariance_Msg_Pub(track_base_map, imgL_raw_ros_msg.header)
        track_stamped_filtered = self.Track_Stamped_With_Covariance_Msg_Pub(track_filtered_map, imgL_raw_ros_msg.header)
        
        self.base_disp_map.publish(base_map)
        self.filtered_disp_map.publish(filtered_map)
        self.Track_Stamped_Base_Pub.publish(track_stamped_base)
        self.Track_Stamped_Filtered_Pub.publish(track_stamped_filtered)
    
    def Track_Stamped_With_Covariance_Msg_Pub(self, cone_track, header):

        track_stamped = TrackStampedWithCovariance()
        track_stamped.header = header
        track_stamped.track = cone_track.track
        self.get_logger().info(f"Número de cones encontrados: {len(cone_track.track)}")
        return track_stamped
    
def main(args=None):
    rclpy.init(args=args)
    cone_track_node = Cone_Track_Process()
    rclpy.spin(cone_track_node)
    cone_track_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    