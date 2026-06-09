#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import select
import tty
import termios

class TecladoCanNode(Node):
    def __init__(self):
        super().__init__('teclado_can_node')
        # Cria o publisher no exato tópico que a sua State Machine está escutando
        self.publisher_ = self.create_publisher(String, '/check_sm/can_msg', 10)
        #self.publisher_ = self.create_publisher(String, '/check_sm/reset', 10)

        self.get_logger().info('--- Nó de Teste Iniciado ---')
        self.get_logger().info('Aperte a tecla "q" para enviar o comando START.')
        self.get_logger().info('Aperte a tecla "r" para enviar o comando RESET.')

        self.get_logger().info('Aperte "Ctrl+C" para sair.')

    def disparar_mensagem(self):
        msg = String()
        msg.data = "START"
        self.publisher_.publish(msg)
        self.get_logger().info('🚀 Mensagem "START" enviada no tópico /check_sm/can_msg!')
        #self.get_logger().info('🚀 Mensagem "RESET" enviada no tópico /check_sm/can_msg!')


# Função para ler uma única tecla do terminal sem precisar dar Enter
def capturar_tecla(settings):
    tty.setraw(sys.stdin.fileno())a
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    rclpy.init(args=args)
    node = TecladoCanNode()
    
    # Salva as configurações originais do terminal
    settings = termios.tcgetattr(sys.stdin)

    try:
        while rclpy.ok():
            tecla = capturar_tecla(settings)
            
            if tecla == 'q' or tecla == 'Q':
                node.disparar_mensagem()
            elif tecla == '\x03': # Código hexadecimal para Ctrl+C
                break
                
    except Exception as e:
        print(e)
    finally:
        # Restaura o terminal para o normal antes de fechar
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()