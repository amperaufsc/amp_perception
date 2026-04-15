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

bridge = CvBridge()

class Cone_Track_Process(Node):

    def __init__(self):
        super().__init__('cone_track_node')
        self.get_logger().info("Nó foi iniciallizado")
        baseline = 0.15

        self.calc = PerceptionProcess(baseline)
                     
        self.image_left_sub = Subscriber(self, Image, "/image_rect/left")
        self.image_right_sub = Subscriber(self, Image, "/image_rect/right")
        self.yolo_inf_sub = Subscriber(self, Yolov8Inference, "/inferenceresult")
        self.base_disp_map = Subscriber(self, Image, "/disparity_map/teste")

        self.Track_Stamped_Base_Pub = self.create_publisher(TrackStampedWithCovariance, "/track_pub/patinho",10)

        max_delay = 1.0
        self.time_sync = ApproximateTimeSynchronizer([self.image_left_sub, self.image_right_sub, self.yolo_inf_sub, self.base_disp_map],10,max_delay)
        self.time_sync.registerCallback(self.sync_callback)
        
        self.get_logger().info("init finalizado")
        

    def sync_callback(self, imgL_raw_ros_msg, imgR_raw_ros_msg, yoloinference, disp_map):
        disp_map = bridge.imgmsg_to_cv2(disp_map, desired_encoding="passthrough")
        track_base_map = self.calc.object_on_map(yoloinference, disp_map, imgL_raw_ros_msg, imgR_raw_ros_msg)
        is_disp_map = track_base_map[1]
        track = track_base_map[0]

        if is_disp_map:
            self.get_logger().warn("Patinho Disparity Map")
            for cone in track.track:
                x = cone.location.x
                y = cone.location.y
                z = cone.location.z
                cone_location = "X = %2fm, Y = %2fm, Z = %2fm" 
                self.get_logger().info(cone_location %(x,y,z))

        else:
            self.get_logger().warn("Patinho Depth Map")
            for cone in track.track:
                x = cone.location.x
                y = cone.location.y
                z = cone.location.z
                cone_location = "X = %2fm, Y = %2fm, Z = %2fm" 
                self.get_logger().info(cone_location %(x,y,z))

        self.Track_Stamped_Base_Pub.publish(track)
    
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
    