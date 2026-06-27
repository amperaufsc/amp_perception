#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode, Node, State, TransitionCallbackReturn
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import Image, CameraInfo
from yolov8_msgs.msg import Yolov8Inference
from stereo_msgs.msg import DisparityImage
from fs_msgs.msg import Track, TrackStampedWithCovariance, Cone, ConeWithCovariance
from std_msgs.msg import String
from lifecycle_msgs.msg import Transition, TransitionEvent
from cv_bridge import CvBridge
import sys
import yaml

from position_estimation.disparity_estimator import DisparityEstimator

bridge = CvBridge()

class Lifecycle_Perception(LifecycleNode):
    def __init__(self):
        super().__init__('perception_lifecycle_node')
        self.get_logger().info("Init done: Perception Unconfigured")

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        try:
            self.get_logger().info("Configuring Perception")
            
            ## Cria os subscribers
            self.img_l_msg = Subscriber(self, Image, "camera/left")
            self.img_R_msg = Subscriber(self, Image, "camera/right")
            self.disparity_msg = Subscriber(self, Image, "disparity")
            self.inference = Subscriber(self, Yolov8Inference, "inference")

            # Cria os lifecycle publishers
            self.track = self.create_lifecycle_publisher(TrackStampedWithCovariance, "track", 10)

            # Declara todos os parâmetros utilizados.
            self.declare_parameter("left_camera_info", "/src/amp_perception/perception/config/OAKDLR_left.yaml")
            self.declare_parameter("right_camera_info", "/src/amp_perception/perception/config/OAKDLR_right.yaml")
            self.declare_parameter("set_disparity", True)

            # Pega os parâmetros
            self.set_disparity = self.get_parameter("set_disparity").value
            self.left_path = self.get_parameter("left_camera_info").value
            self.right_path = self.get_parameter("right_camera_info").value

            with open(self.left_path) as arquivo:
                self.left_camera_info = yaml.load(arquivo, Loader=yaml.FullLoader)

            with open(self.right_path) as arquivo:
                self.right_camera_info = yaml.load(arquivo,Loader=yaml.FullLoader)

            # Instancia da classe de metodos do perception
            self.perception_methods = DisparityEstimator(self.left_camera_info, self.set_disparity)

            return TransitionCallbackReturn.SUCCESS
        
        except Exception as e:
            self.get_logger().warn(f"Configuration failed: {e}")
            return TransitionCallbackReturn.ERROR

    def on_activate(self, state: State) -> TransitionCallbackReturn:        
        try:
            self.get_logger().info("Activating Perception")

            # Chama o on_activate da classe mãe para ativar os lifecycle_publishers
            super().on_activate(state)

            # Garante a sincronização das mensagens
            queue_size = 10
            max_delay = 1
            self.time_sync = ApproximateTimeSynchronizer([self.img_l_msg, 
                                                         self.img_R_msg,
                                                         self.disparity_msg,
                                                         self.inference], 
                                                         queue_size, max_delay)
            
            # Registra o callback
            self.time_sync.registerCallback(self.callback)  
            self.get_logger().warn("Perception activated and ready to process data")
            
            return TransitionCallbackReturn.SUCCESS
        
        except Exception as e:
            self.get_logger().warn(f"Activation failed: {e}")
            return TransitionCallbackReturn.ERROR
        
    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        try:    
            self.get_logger().info("Encerrando o nó de percepção com segurança...")
            
            # O on_shutdown pode ser chamado de qualquer estado (inclusive se o nó estiver ativo).
            # Por segurança, garantimos que o publicador seja destruído se ainda existir.
            if hasattr(self, 'track') and self.track is not None:
                self.destroy_publisher(self.track)
            
            # Limpa referências remanescentes na memória
            self.img_l_msg = None
            self.img_R_msg = None
            self.disparity_msg = None
            self.inference = None
            self.time_sync = None
            self.perception_methods = None
                
            return TransitionCallbackReturn.SUCCESS
        
        except Exception as e:
            self.get_logger().warn(f"Shutdown failed: {e}")
            return TransitionCallbackReturn.ERROR

    def callback(self, img_left_msg, img_right_msg, disparity, inference):
        self.get_logger().info("Callback called with synchronized messages")
        try:
            # Parametros
            focal_length = 453.6716
            baseline = 0.15

            cv2disp_map = bridge.imgmsg_to_cv2(disparity)
            cv2img_left = bridge.imgmsg_to_cv2(img_left_msg)

            # Cria a track
            track = self.perception_methods.get_object_on_map(cv2img_left, cv2disp_map, inference.yolov8_inference, baseline, focal_length)

            # Compoe a mensagem de track
            track = self.perception_methods.track_stamped_with_covariance_msg_compose(track, img_left_msg.header)

            # Publica a track
            self.track.publish(track)

        except Exception as e:
            self.get_logger().warn(f"Shutdown failed: {e}")
            return TransitionCallbackReturn.ERROR
        
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        try:
            self.get_logger().info("Limpando recursos (Unconfiguring)...")
            
            # 1. Destrói explicitamente o publicador lifecycle para liberar a rede do ROS
            if hasattr(self, 'track') and self.track is not None:
                self.destroy_publisher(self.track)
                self.track = None

            # 2. Desvincula os assinantes do message_filters 
            # (O garbage collector do Python finaliza a destruição)
            self.img_l_msg = None
            self.img_R_msg = None
            self.disparity_msg = None
            self.inference = None

            # 3. Limpa instâncias de processamento e sincronizador
            self.time_sync = None
            self.perception_methods = None

            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().warn(f"Cleanup failed: {e}")
            return TransitionCallbackReturn.ERROR

def main(args=None):
    rclpy.init(args=args)
    node = Lifecycle_Perception()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Destrói o nó e encerra o contexto ROS
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()