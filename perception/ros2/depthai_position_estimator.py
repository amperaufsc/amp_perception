#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from std_msgs.msg import Header
from message_filters import Subscriber, ApproximateTimeSynchronizer
from fs_msgs.msg import TrackStampedWithCovariance, TrackStamped
from fs_msgs.msg import Cone
from yolov8_msgs.msg import InferenceResult
from yolov8_msgs.msg import Yolov8Inference
from stereo_msgs.msg._disparity_image import DisparityImage
import sensor_msgs_py.point_cloud2 as pc2
from position_estimation.disparity_estimator import DisparityEstimator
import yaml

bridge = CvBridge()

class PositionEstimator(Node):

    def __init__(self):
        super().__init__('depthai_position_estimator')
        
        self.disparity_sub = Subscriber(self,Image,"disparity")
        self.detections_sub = Subscriber(self, Yolov8Inference, "inference")
        self.image_left_sub = Subscriber(self, Image, "camera/left")
        self.image_right_sub = Subscriber(self, Image, "camera/right")
        self.publishers_ = self.create_publisher(TrackStampedWithCovariance, 'track',10)
        
        queue_size = 10
        max_delay = 1
        self.time_sync = ApproximateTimeSynchronizer([self.image_left_sub,self.detections_sub,self.disparity_sub],queue_size,max_delay)
        self.time_sync.registerCallback(self.sync_callback)

        self.declare_parameter('set_disparity', True)
        
        self.declare_parameter('left_camera_info','src/as_amp/perception/config/OAKDLR_left.yaml')
        self.declare_parameter('right_camera_info','src/as_amp/perception/config/OAKDLR_right.yaml')

        self.left_path = self.get_parameter('left_camera_info').value
        self.right_path = self.get_parameter('right_camera_info').value

        self.set_disparity = self.get_parameter('set_disparity').value

        self.get_logger().info(self.left_path)
        with open(self.left_path) as arquivo:
            self.left_camera_info = yaml.load(arquivo, Loader=yaml.FullLoader)

        with open(self.right_path) as arquivo:
            self.right_camera_info = yaml.load(arquivo,Loader=yaml.FullLoader)

        self.disparity=DisparityEstimator(self.left_camera_info, self.set_disparity)


    def sync_callback(self, img_left,yolo_result,disp_map):

        disp_map_t = 0.150
        disp_map_f = 454.48095703125 

        cv2disp_map=bridge.imgmsg_to_cv2(disp_map)
        cv2img_left=bridge.imgmsg_to_cv2(img_left)
        
        track=self.disparity.get_object_on_map(cv2img_left,cv2disp_map,yolo_result.yolov8_inference,disp_map_t,disp_map_f)
        track=self.track_to_trackstamped_with_covariance(track,img_left.header)
        self.publishers_.publish(track)

    # def track_to_trackstamped(self,track,header):
    #     trackstamped=TrackStamped()
        
    #     trackstamped.header=header
    #     #trackstamped.header.frame_id = self.get_parameter('frame_id').value
    #     trackstamped.track=track.track
    #     return trackstamped
        
    def track_to_trackstamped_with_covariance(self,track,header):
        trackstampedwithcovariance=TrackStampedWithCovariance()
        
        trackstampedwithcovariance.header=header
        trackstampedwithcovariance.track=track.track
        return trackstampedwithcovariance
    
def main(args=None):
    
    rclpy.init(args=args)
    position_estimator = PositionEstimator()
    rclpy.spin(position_estimator)

    position_estimator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    