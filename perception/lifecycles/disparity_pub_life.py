#!/usr/bin/env python3

import numpy as np
import cv2
import time
import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from std_msgs.msg import Header
from message_filters import Subscriber, ApproximateTimeSynchronizer
from utils.perception_calc import PerceptionProcess

bridge = CvBridge()

class Disparity_Publisher(LifecycleNode):

    def __init__(self):
        super().__init__('disparity_pub_life')
        self.get_logger().info("Nó de Disparidade inicializado (Estado: Unconfigured)")

        # Variáveis de controle de desempenho e estado
        self.periodo = 0
        self.total = 0
        self.is_active_flag = False
        
        # Deixamos os objetos vazios para serem criados no on_configure
        self.calc = None
        self.time_sync = None

        # 1. DECLARAÇÃO DOS PARÂMETROS
        self.declare_parameter("left_config_file_name", "OAKDLR_left_22_04.yaml")
        self.declare_parameter("right_config_file_name", "OAKDLR_right_22_04.yaml")
        self.declare_parameter("baseline", 0.15)
        

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        try:
            # Corrigido o typo: self.get_lgger() -> self.get_logger()
            self.get_logger().info("Configurando parâmetros e tópicos...")

            # 2. LEITURA DOS PARÂMETROS GERAIS
            left_config_file_name = self.get_parameter("left_config_file_name").value
            right_config_file_name = self.get_parameter("right_config_file_name").value
            baseline = self.get_parameter("baseline").value

            # Instancia o algoritmo de percepção
            self.calc = PerceptionProcess(baseline, left_config_file_name, right_config_file_name)

            # 3. SUBSCRIBERS USANDO OS TÓPICOS PARAMETRIZADOS
            self.img_left = Subscriber(self, Image, "camera/left", qos_profile=qos_profile_sensor_data)
            self.img_right = Subscriber(self, Image, "camera/right", qos_profile=qos_profile_sensor_data)

            # 4. LIFECYCLE PUBLISHER USANDO O TÓPICO PARAMETRIZADO
            self.disp_patinho_map = self.create_lifecycle_publisher(Image, "disparity", 10)

            # Sincronizador nasce no on_configure para evitar fila suja
            max_delay = 0.1
            self.time_sync = ApproximateTimeSynchronizer([self.img_left, self.img_right], 10, max_delay)
            self.time_sync.registerCallback(self.sync_callback)
            
            self.get_logger().info(f"Ouvindo: {left_topic} e {right_topic}. Publicando em: {disparity_topic}")
            self.get_logger().info("Configuração finalizada com sucesso.")
            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.get_logger().error(f"Falha ao configurar: {e}")
            return TransitionCallbackReturn.ERROR

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Ativando disparidade (Publicadores abertos)...")
        # Ativa os lifecycle publishers da classe mãe
        super().on_activate(state)
        # Libera o callback para realizar o cálculo estéreo
        self.is_active_flag = True
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Desativando disparidade (Pausando cálculos)...")
        # Bloqueia o callback imediatamente para poupar CPU
        self.is_active_flag = False
        super().on_deactivate(state)
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Limpando memória (Unconfiguring)...")
        self.destroy_publisher(self.disp_patinho_map)
        self.calc = None
        self.time_sync = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Encerrando o nó de disparidade de forma segura.")
        self.is_active_flag = False
        return TransitionCallbackReturn.SUCCESS

    def sync_callback(self, img_L, img_R):
        # TRAVA DE SEGURANÇA: Impede processamento se não estiver no estado ACTIVE
        if not self.is_active_flag:
            return

        start_time = time.time()
        
        # Processamento das imagens
        img_L_rect_msg, img_R_rect_msg = self.calc.approximate_stereo_rectify(img_L, img_R)
        disp_map = self.calc.DisparityProcess(img_L_rect_msg, img_R_rect_msg)[1]

        # Conversão e repasse de headers
        disp_map = bridge.cv2_to_imgmsg(disp_map)
        disp_map.header = img_L.header

        # img_L_rect_msg.header = img_L.header
        # img_R_rect_msg.header = img_R.header
        
        self.disp_patinho_map.publish(disp_map)
            
        end_time = time.time()
        self.total += end_time - start_time
        self.periodo += 1
        periodo_medio = (self.total/self.periodo)

        self.get_logger().info(f"Frequencia media do Callback: {1/periodo_medio:.4f} hz ")

    def draw_epilines(self, imgL_rect, imgR_rect):
        n_lines = 20
        imgL_rect = bridge.imgmsg_to_cv2(imgL_rect)
        imgR_rect = bridge.imgmsg_to_cv2(imgR_rect)

        h, w = imgL_rect.shape[:2]
        combined = np.hstack([imgL_rect, imgR_rect])
        
        for y in range(0, h, h // n_lines):
            cv2.line(combined, (0, y), (w * 2, y), (0, 255, 0), 1)

        return combined

def main(args=None):
    rclpy.init(args=args)
    
    disparity_map_pub = Disparity_Publisher()
    
    try:
        rclpy.spin(disparity_map_pub)
    except KeyboardInterrupt:
        pass
    finally:
        disparity_map_pub.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()