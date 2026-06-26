#!/usr/bin/env python3
from __future__ import print_function

import numpy as np
import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import qos_profile_sensor_data
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

class Cone_Track_Process(LifecycleNode):

    def __init__(self):
        super().__init__('track_node_publisher')
        self.get_logger().info("Nó Lifecycle inicializado (Estado: Unconfigured)")
        
        # Variáveis de estado e métricas
        self.total = 0
        self.periodo = 0
        self.is_active_flag = False
        
        # Deixamos os objetos vazios na inicialização
        self.calc = None
        self.time_sync = None

        # 1. DECLARAÇÃO DOS PARÂMETROS COM OS VALORES PADRÃO
        self.declare_parameter("left_config_file_name", "OAKDLR_left_22_04.yaml")
        self.declare_parameter("right_config_file_name", "OAKDLR_right_22_04.yaml")
        self.declare_parameter("baseline", 0.15)

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        try:
            self.get_logger().info("Configurando o nó de percepção...")

            # 2. LEITURA DOS VALORES (Pega o que veio do launch file, ou usa o padrão)
            left_config_file_name = self.get_parameter("left_config_file_name").value
            right_config_file_name = self.get_parameter("right_config_file_name").value
            baseline = self.get_parameter("baseline").value

            # Instancia o processo com os parâmetros capturados
            self.calc = PerceptionProcess(baseline, left_config_file_name, right_config_file_name)
                         
            # Aplicando QoS de sensor para garantir que mensagens pesadas cheguem sem travar a rede
            self.image_left_sub = Subscriber(self, Image, "/oak/left/image_raw", qos_profile=qos_profile_sensor_data)
            self.image_right_sub = Subscriber(self, Image, "/oak/right/image_raw", qos_profile=qos_profile_sensor_data)
            self.base_disp_map = Subscriber(self, Image, "/oak/stereo/image_raw", qos_profile=qos_profile_sensor_data)
            self.yolo_inf_sub = Subscriber(self, Yolov8Inference, "inferenceresult")

            # ATENÇÃO: Mudança para create_lifecycle_publisher
            self.Track_Stamped_Base_Pub = self.create_lifecycle_publisher(TrackStampedWithCovariance, "track", 10)
            
            # Sincronizador nasce junto com os tópicos no on_configure
            max_delay = 0.1
            self.time_sync = ApproximateTimeSynchronizer(
                [self.image_left_sub, self.image_right_sub, self.yolo_inf_sub, self.base_disp_map],
                30, 
                max_delay
            )
            self.time_sync.registerCallback(self.sync_callback)
            
            self.get_logger().info("Configuração finalizada com sucesso.")
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f"Falha ao configurar: {e}")
            return TransitionCallbackReturn.ERROR

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Ativando o nó e publicadores...")
        # Ativa os lifecycle publishers
        super().on_activate(state)
        # Libera o callback para processar as imagens
        self.is_active_flag = True
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Desativando o nó (Pausando processamento)...")
        # Trava o callback imediatamente
        self.is_active_flag = False
        # Desativa os publishers
        super().on_deactivate(state)
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Limpando recursos (Unconfiguring)...")
        # Destrói os publicadores para liberar memória
        self.destroy_publisher(self.Track_Stamped_Base_Pub)
        self.calc = None
        self.time_sync = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Encerrando o nó com segurança...")
        self.is_active_flag = False
        return TransitionCallbackReturn.SUCCESS

    def sync_callback(self, imgL_raw_ros_msg, imgR_raw_ros_msg, yoloinference, disp_map):
        # TRAVA DE SEGURANÇA: Se o nó não estiver ativado, o message_filters descarta os frames e não processa.
        if not self.is_active_flag:
            return

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
    
    try:
        rclpy.spin(cone_track_node)
    except KeyboardInterrupt:
        pass
    finally:
        cone_track_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()