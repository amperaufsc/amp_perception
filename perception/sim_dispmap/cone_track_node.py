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
from fs_msgs.msg import TrackStamped
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
                
        self.image_left_sub = Subscriber(self, Image, "/fsds/cam2/image_color")
        self.image_right_sub = Subscriber(self, Image, "/fsds/cam1/image_color")
        self.yolo_inf_sub = Subscriber(self, Yolov8Inference, "/inferenceresult")
        self.gps_sim_sub = Subscriber(self, NavSatFix, "/fsds/gps")

        self.Track_Stamped_Pub = self.create_publisher(TrackStamped, "/track_pub/trackstamped",10)
        
        max_delay = 0.1
        self.time_sync = ApproximateTimeSynchronizer([self.image_left_sub,self.image_right_sub,self.yolo_inf_sub, self.gps_sim_sub],10,max_delay)
        self.time_sync.registerCallback(self.sync_callback)
        
        self.get_logger().info("init finalizado")
        

    def sync_callback(self, imgL_raw_ros_msg, imgR_raw_ros_msg, yoloinference, gps_sim):
        self.get_logger().info("callback")
        disp_map= self.calc.DisparityProcess(imgL_raw_ros_msg, imgR_raw_ros_msg)
        track = self.calc.object_on_map(yoloinference, disp_map, imgL_raw_ros_msg, imgR_raw_ros_msg)[0]
        disparity = self.calc.object_on_map(yoloinference, disp_map, imgL_raw_ros_msg, imgR_raw_ros_msg)[1]
        self.get_logger().info(f"diparidade: {disparity}")
        track_stamped = self.Track_Stamped_Msg_Pub(track, gps_sim.header)
        self.Track_Stamped_Pub.publish(track_stamped)
    
    def Track_Stamped_Msg_Pub(self, cone_track, header):
        
        track_stamped = TrackStamped()
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
    