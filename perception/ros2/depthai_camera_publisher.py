#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Header
from fs_msgs.msg import Track
from fs_msgs.msg import Cone
from stereo_msgs.msg._disparity_image import DisparityImage
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from position_estimation.depthai_camera_setup import depthai_camera_setup

nnBlobPath = '/home/jetson/ws/src/as_amp/perception/position_estimation/best_nano_openvino_2022.1_6shave.blob'

class depthai_camera_publisher(Node):
    def __init__(self):                                 
        super().__init__('depthai_position_estimator')

        self.get_logger().info("Depthai Working")       
                                                
        # ROS2 publishers
        self.rgb_publisher = self.create_publisher(Image, '/camera/rgb/image_raw', 10)
        self.disparity_map = self.create_publisher(DisparityImage, '/disparity_msg', 10)
        #self.detection_publisher = self.create_publisher(Track, '/position_estimation/track_detections', 10)
        #self.position_publisher = self.create_publisher(Cone, '/cone_3d_positions', 10) 
        #self.publishers_point_clound = self.create_publisher(PointCloud2, '/position_estimation/point_clound',10)
        self.monoLeft_publisher = self.create_publisher(Image, '/camera/left/image_raw',10)
        self.monoRight_publisher = self.create_publisher(Image, '/camera/right/image_raw',10)

        self.yolo = depthai_camera_setup()

        self.timer = self.create_timer(0.1, self.sync_callback)

        self.bridge = CvBridge()

        self.a = False
        self.b = False

    def sync_callback (self):
        #Callback para capturar dados do YOLO e publicar nos tópicos ROS2.
        
        #self.get_logger().info("Depthai Callback Working")

        #self.a = self.yolo.test(self.a)

        #self.get_logger().info(f"{self.a}")
        # Obter dados do YOLO
        depthFrame, inLeftFrame, inRightFrame = self.yolo.process_data()

        #self.get_logger().info(f"{self.b}")

        #self.get_logger().info("Depthai Callback Working PT2")

        intrinsic_matrix = self.yolo.get_image_with_intrinsics()

        # Publicar frame RGB
        #rgb_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        #self.rgb_publisher.publish(rgb_msg)

        # Receber mensagens em ROS
        disparity, Left_msg, Right_msg = self.std_msgs_publishers(
            depthFrame, intrinsic_matrix, inLeftFrame, inRightFrame
        )

        self.monoLeft_publisher.publish(Left_msg)

        self.monoRight_publisher.publish(Right_msg)

        # Publicar frame de profundidade (bruto)
        self.disparity_map.publish(disparity)
        
        '''
        # Publicar mensagem de detecção
        detection_msg = track
        if len(detection_msg.track) > 0:
            self.detection_publisher.publish(detection_msg)
        
        # Publicar mensagens de pointcloud
        self.publishers_point_clound.publish(pointclound)
        '''
        #self.get_logger().info("Depthai Callback Working PT2")

    def std_msgs_publishers(self,depthFrame, intrinsic_matrix, inRightFrame, inLeftFrame):
        
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "camera_link" 

        ###
        '''
        points = []
        for cone in track_msg.track: 
            x = cone.location.x
            y = cone.location.y
            z = cone.location.z
            points.append([x, y, z])

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        pointcloud_msg = pc2.create_cloud(header, fields, points)
        '''
        ###

        disparity_msg = DisparityImage()
        disparity_msg.image = self.bridge.cv2_to_imgmsg(depthFrame, encoding="mono16")
        disparity_msg.f = float(intrinsic_matrix[0][0][0])
        disparity_msg.t = 0.075 
        
        ###

        right_image_msg = self.bridge.cv2_to_imgmsg(inRightFrame)
        right_image_msg.header = header
        
        right_image_msg.is_bigendian = 0
        right_image_msg.height = 780
        right_image_msg.width = 1200
        right_image_msg.encoding = "bgr8"
        
        ###

        left_image_msg = self.bridge.cv2_to_imgmsg(inLeftFrame)
        left_image_msg.header = header
        
        left_image_msg.is_bigendian = 0
        left_image_msg.height = 780
        left_image_msg.width = 1200
        left_image_msg.encoding = "bgr8"

        return disparity_msg, right_image_msg, left_image_msg


def main(args=None):
    
    rclpy.init(args=args)
    camera_publisher = depthai_camera_publisher()
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()