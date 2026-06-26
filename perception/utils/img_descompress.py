#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2

class StereoDecompressorNode(Node):
    def __init__(self):
        super().__init__('stereo_decompressor_node')
        self.bridge = CvBridge()

        # =====================================================================
        # PUBLISHERS: Tópicos Raw de saída (Já em tons de cinza / mono8)
        # =====================================================================
        self.pub_left = self.create_publisher(Image, '/oak/left/image', 10)
        self.pub_right = self.create_publisher(Image, '/oak/right/image', 10)

        # =====================================================================
        # SUBSCRIBERS: Tópicos de entrada comprimida
        # =====================================================================
        # Nota: Não usamos message_filters aqui para máxima velocidade. 
        # Descomprimimos assim que o pacote chega. O nó do SGBM sincroniza depois.
        self.sub_left = self.create_subscription(
            CompressedImage,
            '/oak/left/image_raw/compressed',
            self.left_callback,
            qos_profile_sensor_data)

        self.sub_right = self.create_subscription(
            CompressedImage,
            '/oak/right/image_raw/compressed',
            self.right_callback,
            qos_profile_sensor_data)

        self.get_logger().info("Nó Descompressor Estéreo Iniciado! Convertendo: Compressed -> Raw (Mono8)")

    def process_and_publish(self, msg, publisher):
        # Lógica preguiçosa: Só gasta CPU descomprimindo se o nó do SGBM estiver ligado pedindo imagens
        if publisher.get_subscription_count() == 0:
            return

        try:
            # 1. Extrai a imagem comprimida (ignora os metadados chatos do ROS)
            cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")

            # 2. Vacina contra o erro do "8UC3" -> Força para tons de cinza se for colorida
            if len(cv_img.shape) == 3:
                cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

            # 3. Empacota de volta como Imagem Raw, cravando que é "mono8"
            raw_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding="mono8")
            
            # 4. REGRA DE OURO: Copia o carimbo de tempo (Timestamp) original!
            # Sem isso, o nó SGBM não vai conseguir achar o par (esquerda/direita) da mesma fração de segundo.
            raw_msg.header = msg.header 

            # 5. Publica
            publisher.publish(raw_msg)

        except Exception as e:
            self.get_logger().error(f"Erro crítico ao descomprimir a imagem: {e}")

    def left_callback(self, msg):
        self.process_and_publish(msg, self.pub_left)

    def right_callback(self, msg):
        self.process_and_publish(msg, self.pub_right)

def main(args=None):
    rclpy.init(args=args)
    node = StereoDecompressorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()