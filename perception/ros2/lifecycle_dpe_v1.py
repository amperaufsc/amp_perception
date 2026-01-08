#!/usr/bin/env python3

# from ultralytics import YOLO
import rclpy
from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from message_filters import Subscriber, ApproximateTimeSynchronizer
from fs_msgs.msg import Track
from fs_msgs.msg import ConeWithCovariance, Cone
from fs_msgs.msg import TrackStampedWithCovariance, TrackStamped
from yolov8_msgs.msg import InferenceResult
from yolov8_msgs.msg import Yolov8Inference
from stereo_msgs.msg._disparity_image import DisparityImage
import sensor_msgs_py.point_cloud2 as pc2
from position_estimation.disparity_estimator import DisparityEstimator
import yaml
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
import yaml
from lifecycle_msgs.msg import Transition, TransitionEvent
from lifecycle_msgs.srv import ChangeState
import sys
from std_msgs.msg import String

bridge = CvBridge()


class PositionEstimator(LifecycleNode):

    def __init__(self):
        super().__init__('position_estimator')
        
        
        self.queue_size = 10
        self.max_delay = 1

        # ESSA MERDA DO CAMINHO DO PC DO GARBIN NAO NAO SEI SE MANTENHO ASSIM
        # self.declare_parameter('left_camera_info','/home/pedro_garbin/ws/src/as_amp/perception/config/fsds_left.yaml')
        # self.declare_parameter('right_camera_info','/home/pedro_garbin/ws/src/as_amp/perception/config/fsds_right.yaml')

    def on_shutdown(self, state: LifecycleState):
        self.get_logger().info("SHUTDOWN Perception")
        return TransitionCallbackReturn.SUCCESS
    
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring Perception...')

        try:

        # Declaração dos parâmetros
            self.declare_parameter('set_disparity', True)
            self.declare_parameter('frame_id','frame_id')
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
            

            # Criando os publishers
            self.publishers_ = self.create_publisher(TrackStamped, 'track',10) #Mudar pra TrackWithCovariance
            # Publisher de mudanca de estado do state_machine
            self.transition_pub_emergency = self.create_lifecycle_publisher(TransitionEvent, 'EbsNOTMissionFinished', 10)

            self.disparity_sub = Subscriber(self,DisparityImage,"disparity")
            self.detections_sub = Subscriber(self, Yolov8Inference, "inference")
            self.image_left_sub = Subscriber(self, Image, "camera/left")
            self.image_right_sub = Subscriber(self, Image, "camera/right")

            self.publishers_point_clound = self.create_publisher(PointCloud2, 'pointcloud',10)

            # Publisher resposnavel pela ativação dos leds do carro baseado em seu estado
            self.led_pub = self.create_publisher(String, '/AMP/as_status_indicator', 10)
            self.led_msg = String()

            self.get_logger().info('Configuração PERCEPTION concluída com sucesso.')
            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
                self.get_logger().error(f'Erro durante on_configure: {e}')
                return TransitionCallbackReturn.FAILURE
    
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Activating PerceptionNode...')

        try:
            queue_size = 10
            max_delay = 1
            self.time_sync = ApproximateTimeSynchronizer([self.image_left_sub,self.detections_sub,
                                                self.disparity_sub],self.queue_size,self.max_delay)

            self.time_sync.registerCallback(self.sync_callback)

            self.get_logger().info('PerceptionNode ativado com sucesso.')

            
            return super().on_activate(state)

        except Exception as e:
            self.get_logger().error(f'Erro durante on_activate: {e}')
        return TransitionCallbackReturn.FAILURE
    
    def sync_callback(self, img_left,yolo_result,disp_map):

        try:
            disp_map_t = 0.150
            disp_map_f = 454.48095703125 

            cv2disp_map=bridge.imgmsg_to_cv2(disp_map.image)
            cv2img_left=bridge.imgmsg_to_cv2(img_left)
        
            track=self.disparity.get_object_on_map(cv2img_left,cv2disp_map,yolo_result.yolov8_inference,disp_map.t,disp_map.f)
            track=self.track_to_trackstamped_with_covariance(track,img_left.header)
            self.publishers_.publish(track)
            pointclound=self.track_to_point_cloud(track)
            self.publishers_point_clound.publish(pointclound)
        except:
            event = TransitionEvent()
            self.transition_pub_emergency(event) # Muda o estado do carro pra Emergency
            self.led_msg.data = 'as_emergency'
            self.led_pub.publish(self.led_msg) # Publica mensagem de Emergency pra ativação do LED
            sys.exit(1) # Mata o processo com chamada de sistema

    def track_to_point_cloud(self,track_msg):

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.get_parameter('frame_id').value  # Ajuste para o frame de referência correto


        points = []
        for cone in track_msg.track:  # Supondo que track_msg.tracks é a lista de rastreamentos
            x = cone.location.x
            y = cone.location.y
            z = cone.location.z
            points.append([x, y, z])

    # Define os campos da nuvem de pontos
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

    # Cria a mensagem PointCloud2
        pointcloud_msg = pc2.create_cloud(header, fields, points)
        return pointcloud_msg
    
    '''def track_to_trackstamped(self,track,header):
        trackstamped=TrackStamped()
        
        trackstamped.header=header
        trackstamped.header.frame_id = self.get_parameter('frame_id').value
        trackstamped.track=track.track
        return trackstamped'''

    def track_to_trackstamped_with_covariance(self,track,header):
        trackstampedwithcovariance=TrackStampedWithCovariance()
        
        trackstampedwithcovariance.header=header
        trackstampedwithcovariance.header.frame_id = self.get_parameter('frame_id').value
        trackstampedwithcovariance.track=track.track
        #trackstampedwithcovariance.
        return trackstampedwithcovariance


def main(args=None):
    rclpy.init(args=args)


    position_estimator = PositionEstimator()
    #position_estimator.get_logger().info(args)

    rclpy.spin(position_estimator)

    position_estimator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()