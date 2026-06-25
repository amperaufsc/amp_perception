#!/usr/bin/env python3
from __future__ import print_function

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix, CameraInfo
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge
from std_msgs.msg import Header
from message_filters import Subscriber, ApproximateTimeSynchronizer
from fs_msgs.msg import TrackStampedWithCovariance
from perception_calc import PerceptionProcess
from yolov8_msgs.msg import Yolov8Inference
import cv2
import time
bridge = CvBridge()

class Cone_Track_Process(Node):

    def __init__(self):
        super().__init__('cone_track_node')
        self.get_logger().info("Nó foi iniciallizado")

        left_config_file_name = "OAKDLR_left_22_04.yaml"
        right_config_file_name = "OAKDLR_right_22_04.yaml"

        baseline = 0.15

        self.calc = PerceptionProcess(baseline, left_config_file_name, right_config_file_name)
                     
        self.image_left_sub = Subscriber(self, Image, "camera/left")
        self.image_right_sub = Subscriber(self, Image, "camera/right")
        self.yolo_inf_sub = Subscriber(self, Yolov8Inference, "inferenceresult")
        self.base_disp_map = Subscriber(self, Image, "disparity")

        self.Track_Stamped_Base_Pub = self.create_publisher(TrackStampedWithCovariance, "track",10)
        max_delay = 0.1
        self.time_sync = ApproximateTimeSynchronizer([self.image_left_sub, self.image_right_sub, self.yolo_inf_sub, self.base_disp_map],30,max_delay)
        self.time_sync.registerCallback(self.sync_callback)
        
        self.get_logger().info("init finalizado")

        self.total = 0
        self.periodo = 0
        

    def sync_callback(self, imgL_raw_ros_msg, imgR_raw_ros_msg, yoloinference, disp_map):
        start_time = time.time()
        disp_map = bridge.imgmsg_to_cv2(disp_map, desired_encoding="passthrough")
        track_base_map = self.calc.object_on_map(yoloinference, disp_map, imgL_raw_ros_msg, imgR_raw_ros_msg)
        is_disp_map = track_base_map[1]
        track = track_base_map[0]

        if is_disp_map:
            self.get_logger().warn("Disparity Map")
            for cone in track.track:
                x = cone.location.x
                y = cone.location.y
                z = cone.location.z
                cone_location = "X = %2fm, Y = %2fm, Z = %2fm" 
                #self.get_logger().info(cone_location %(x,y,z))

        else:
            self.get_logger().warn("Depth Map")

            for cone in track.track:
                x = cone.location.x
                y = cone.location.y
                z = cone.location.z
                cone_location = "X = %2fm, Y = %2fm, Z = %2fm" 
                #self.get_logger().info(cone_location %(x,y,z))

        self.Track_Stamped_Base_Pub.publish(track)

        end_time = time.time()

        self.total += end_time - start_time
        self.periodo += 1
        periodo_medio = (self.total/self.periodo)

        self.get_logger().info(f"Frequencia media do Callback: {1/periodo_medio:.4f} hz ")
    
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
    