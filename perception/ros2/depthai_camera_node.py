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
from position_estimation.depthai_detection import depthai_detection

class depthai_position_estimator(Node):
    def __init__(self):                                 
        super().__init__('depthai_camera_node')

        self.get_logger().info("Depthai Working")       
                                                
        # ROS2 publishers
        self.rgb_publisher = self.create_publisher(Image, '/camera/rgb/image_raw', 10)
        self.disparity_map = self.create_publisher(DisparityImage, '/disparity_msg', 10)
        self.detection_publisher = self.create_publisher(Track, '/track', 10)
        self.position_publisher = self.create_publisher(Cone, '/cone', 10) 
        self.publishers_point_clound = self.create_publisher(PointCloud2, '/point_clound',10)
        self.monoLeft_publisher = self.create_publisher(Image, '/camera/left/image_raw',10)
        self.monoRight_publisher = self.create_publisher(Image, '/camera/right/image_raw',10)

        self.yolo = depthai_detection()

        self.timer = self.create_timer(0.001, self.sync_callback)

        self.bridge = CvBridge()

    def sync_callback (self):
        #Callback para capturar dados do YOLO e publicar nos tópicos ROS2.
        
        # Obter dados do YOLO
        depthFrame, inLeftFrame, inRightFrame, track = self.yolo.process_data()

        intrinsic_matrix = self.yolo.get_image_with_intrinsics()

        # Receber mensagens em ROS
        disparity, Left_msg, Right_msg = self.std_msgs_publishers(
            track,
            depthFrame, 
            intrinsic_matrix, inLeftFrame, 
            inRightFrame
        )
        self.publish_track_as_pointcloud(track)

        self.monoLeft_publisher.publish(Left_msg)

        self.monoRight_publisher.publish(Right_msg)

        # Publicar frame de profundidade (bruto)
        self.disparity_map.publish(disparity)
        
        # Publicar mensagem de detecção
        detection_msg = track
        if len(detection_msg.track) > 0:
            self.detection_publisher.publish(detection_msg)
            

    def publish_track_as_pointcloud(self,track_msg):
        header= Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "camera_link" 
        points = []
        for cone in track_msg.track: 
            x = float(cone.location.x)
            y = float(cone.location.y)
            z = float(cone.location.z)
            
            points.append([x, y, z])

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        pointcloud_msg = pc2.create_cloud(header, fields, points)
        self.publishers_point_clound.publish(pointcloud_msg)
        
    def std_msgs_publishers(self, track_msg, depthFrame, intrinsic_matrix, inLeftFrame, inRightFrame):
        
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "camera_link" 

        disparity_msg = DisparityImage()
        disparity_msg.image = self.bridge.cv2_to_imgmsg(depthFrame, encoding="mono16")
        disparity_msg.f = float(intrinsic_matrix[0][0][0])
        print(intrinsic_matrix[0][0][0])
        disparity_msg.t = 0.075 

        right_image_msg = self.bridge.cv2_to_imgmsg(inRightFrame)
        right_image_msg.header = header
        
        right_image_msg.is_bigendian = 0
        right_image_msg.height = inRightFrame.shape[0]
        right_image_msg.width = inRightFrame.shape[1]
        right_image_msg.encoding = "bgr8"

        left_image_msg = self.bridge.cv2_to_imgmsg(inLeftFrame)
        left_image_msg.header = header
        
        left_image_msg.is_bigendian = 0
        left_image_msg.height = inLeftFrame.shape[0]
        left_image_msg.width = inLeftFrame.shape[1]
        left_image_msg.encoding = "bgr8"

        return disparity_msg, left_image_msg, right_image_msg
    
def main(args=None):
    
    rclpy.init(args=args)
    position_estimator = depthai_position_estimator()
    rclpy.spin(position_estimator)
    position_estimator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()