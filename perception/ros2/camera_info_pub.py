#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import yaml
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge


left_path = "/home/carlosmello/ws/src/passive_stereo/cfg/fsds_left.yaml"
right_path = "/home/carlosmello/ws/src/passive_stereo/cfg/fsds_right.yaml"
bridge= CvBridge()


class CameraInfoPub(Node):

    def __init__(self):
        super().__init__('camera_info_pub')

        self.publisher_left = self.create_publisher(CameraInfo, '/fsds/cameracam2/camera_info',10)
        self.publisher_right = self.create_publisher(CameraInfo, '/fsds/cameracam1/camera_info',10)


        self.left_camera_info = self.load_camera_info_from_yaml(left_path)
        self.right_camera_info = self.load_camera_info_from_yaml(right_path)

        timer_period = 1 
        self.timer = self.create_timer(timer_period, self.info_callback )



    def info_callback(self):
       
        self.publisher_left.publish(self.left_camera_info)
        self.publisher_right.publish(self.right_camera_info)
    
    def load_camera_info_from_yaml(self, file_path):
        
        try:
            with open(file_path, 'r') as file:
                data = yaml.safe_load(file)

            camera_info = CameraInfo()
            camera_info.width = data['image_width']
            camera_info.height = data['image_height']
            camera_info.k = data['camera_matrix']['data']
            camera_info.d = data['distortion_coefficients']['data']
            camera_info.r = data.get('rectification_matrix', {}).get('data', [0] * 9)
            camera_info.p = data.get('projection_matrix', {}).get('data', [0] * 12)

            return camera_info
        except Exception as e:
            self.get_logger().error(f"Erro ao carregar arquivo YAML: {e}")
            return None


def main(args=None):
    rclpy.init(args=args)

    camera_info_pub = CameraInfoPub()

    rclpy.spin(camera_info_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_info_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()