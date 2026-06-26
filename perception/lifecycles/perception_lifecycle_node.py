#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode, Node, State, TransitionCallbackReturn
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import Image, CameraInfo
from yolov8_msgs.msg import InferenceResult
from stereo_msgs.msg import DisparityImage
from fs_msgs.msg import Track, TrackStampedWithCovariance, Cone, ConeWithCovariance
from std_msgs.msg import String
from lifecycle_msgs.msg import Transition, TransitionEvent
import sys
import yaml

from position_estimation.disparity_estimator import DisparityEstimator

class Lifecycle_Perception(LifecycleNode):
    def __init__(self):
        super().__init__('perception_lifecycle_node')
        self.get_logger().info("Init done: Perception Unconfigured")

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        try:
            self.get_logger().info("Configuring Perception")
            
            ## Cria os subscribers
            self.img_l_msg = Subscriber(self, Image, "/oak/left/image_raw")
            self.img_R_msg = Subscriber(self, Image, "/oak/right/image_raw")
            self.disparity_msg = Subscriber(self, Image, "/oak/stereo/image_raw")
            self.inference = Subscriber(self, InferenceResult, "inference")

            # Cria os lifecycle publishers
            self.track = self.create_lifecycle_publisher(Track, "track", 10)

            # Declara todos os parâmetros utilizados.
            self.declare_parameter("left_camera_info", "src/amp_perception/perception/config/OAKDLR_left.yaml")
            self.declare_parameter("right_camera_info", "src/amp_perception/perception/config/OAKDLR_right.yaml")
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
            
            return TransitionCallbackReturn.SUCCESS
        
        except Exception as e:
            self.get_logger().warn(f"Activation failed: {e}")
            return TransitionCallbackReturn.ERROR
        
    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        try:    
            self.get_logger().info("Exiting node with safety")
            
            # TODO: Garantir que portas seriais ou conexões de rede sejam fechadas    
            return TransitionCallbackReturn.SUCCESS
        
        except Exception as e:
            self.get_logger().warn(f"Shutdown failed: {e}")
            return TransitionCallbackReturn.ERROR

    def callback(self, img_left_msg, img_right_msg, disparity, inference):
        try:
            # Parametros
            focal_length = 453.6716
            baseline = 0.15

            # Cria a track
            track = self.perception_methods.get_object_on_map(img_left_msg, disparity, inference, baseline, focal_length)

            # Compoe a mensagem de track
            track = self.perception_methods.Track_Stamped_With_Covariance_Msg_Compose(track, self.img_l_msg.header)

            # Publica a track
            self.track.publish(track)

        except Exception as e:
            self.get_logger().warn(f"Shutdown failed: {e}")
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