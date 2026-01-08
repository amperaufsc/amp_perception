#!/usr/bin/env python3

import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from std_msgs.msg import Header, String
from message_filters import Subscriber, ApproximateTimeSynchronizer
from fs_msgs.msg import TrackStampedWithCovariance, TrackStamped
from fs_msgs.msg import Cone
from yolov8_msgs.msg import InferenceResult
from yolov8_msgs.msg import Yolov8Inference
from stereo_msgs.msg._disparity_image import DisparityImage
import sensor_msgs_py.point_cloud2 as pc2
from position_estimation.disparity_estimator import DisparityEstimator
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
import yaml
from lifecycle_msgs.msg import Transition, TransitionEvent
from lifecycle_msgs.srv import ChangeState
import sys

bridge = CvBridge()

class PositionEstimator(LifecycleNode):
    def __init__(self):
        super().__init__('perception_node')

    def change_own_state(self, transition_id): #obrigado chatgpt
        client = self.create_client(ChangeState, f'/{self.node_name}/change_state')

        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Serviço de mudança de estado não está disponível.")
            return

        req = ChangeState.Request()
        req.transition.id = transition_id

        future = client.call_async(req)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Transição feita com sucesso: {future.result().success}")
        else:
            self.get_logger().error("Falha ao fazer a transição de estado.")    

    def on_shutdown(self, state: LifecycleState):
        self.get_logger().info("SHUTDOWN Perception")
        return TransitionCallbackReturn.SUCCESS
    
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring Perception...')

        try:

        # Declaração dos parâmetros
    
            self.declare_parameter('set_disparity', True)
            
            self.declare_parameter('left_camera_info','src/as_amp/perception/config/OAKDLR_left.yaml')
            self.declare_parameter('right_camera_info','src/as_amp/perception/config/OAKDLR_right.yaml')

            self.left_path = self.get_parameter('left_camera_info').value
            self.right_path = self.get_parameter('right_camera_info').value

            self.set_disparity = self.get_parameter('set_disparity').value

            # self.get_logger().info(self.left_path)
            with open(self.left_path) as arquivo:
                self.left_camera_info = yaml.load(arquivo, Loader=yaml.FullLoader)

            with open(self.right_path) as arquivo:
                self.right_camera_info = yaml.load(arquivo,Loader=yaml.FullLoader)

            self.disparity=DisparityEstimator(self.left_camera_info, self.set_disparity)
            

            # Criando os publishers
            self.publishers_ = self.create_publisher(TrackStamped, 'track_ALTERA_DPS',10)
            # Publisher de mudanca de estado do state_machine
            self.transition_pub_emergency = self.create_lifecycle_publisher(TransitionEvent, 'EbsNOTMissionFinished', 10)

            self.disparity_sub = Subscriber(self, Image, "disparity")
            self.detections_sub = Subscriber(self, Yolov8Inference, "inference")
            self.image_left_sub = Subscriber(self, Image, "camera/left")
            self.image_right_sub = Subscriber(self, Image, "camera/right")

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
            self.time_sync = ApproximateTimeSynchronizer([self.image_left_sub,self.detections_sub,self.disparity_sub],queue_size,max_delay)
            self.time_sync.registerCallback(self.sync_callback)

            # Criando o sincronizador
            self.time_sync = ApproximateTimeSynchronizer(
                [self.image_left_sub, self.detections_sub, self.disparity_sub],
                queue_size=10,
                slop=1.0
            )
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

            cv2disp_map=bridge.imgmsg_to_cv2(disp_map)
            cv2img_left=bridge.imgmsg_to_cv2(img_left)
            
            track=self.disparity.get_object_on_map(cv2img_left,cv2disp_map,yolo_result.yolov8_inference,disp_map_t,disp_map_f)
            track=self.track_to_trackstamped(track,img_left.header)
            self.publishers_.publish(track)
        except:
            event = TransitionEvent()

            self.transition_pub_emergency(event) # Muda o estado do carro pra Emergency
            self.led_msg.data = 'as_emergency'
            self.led_pub.publish(self.led_msg) # Publica mensagem de Emergency pra ativação do LED
            sys.exit(1) # Mata o processo com chamada de sistema
        
    def track_to_trackstamped(self,track,header):
        trackstampedwithcovariance=TrackStamped()
        
        trackstampedwithcovariance.header=header
        trackstampedwithcovariance.track=track.track
        return trackstampedwithcovariance
    
    def change_own_state(self, transition_id): #obrigado chatgpt
        client = self.create_client(ChangeState, f'/{self.node_name}/change_state')

        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Serviço de mudança de estado não está disponível.")
            sys.exit(1)
            return

        req = ChangeState.Request()
        req.transition.id = transition_id

        future = client.call_async(req)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Transição feita com sucesso: {future.result().success}")
        else:
            self.get_logger().error("Falha ao fazer a transição de estado.")    

def main(args=None):
    
    rclpy.init(args=args)
    position_estimator = PositionEstimator()
    rclpy.spin(position_estimator)

    position_estimator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()