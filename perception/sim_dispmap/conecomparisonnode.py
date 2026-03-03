#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer
import matplotlib.pyplot as plt
from fs_msgs.msg import TrackStampedWithCovariance

class DepthValidatorNode(Node):
    def __init__(self):
        super().__init__('depth_validator_node')

        # Subscribers
        self.sub_t1 = Subscriber(self, TrackStampedWithCovariance, '/track_pub/patinho')
        self.sub_t2 = Subscriber(self, TrackStampedWithCovariance, '/track_pub/luxonis')

        # Sincronização
        self.ts = ApproximateTimeSynchronizer([self.sub_t1, self.sub_t2], 10, 10)
        self.ts.registerCallback(self.callback)

        # Configuração: 2 subplots (Mapa 2D e Gráfico de Profundidade)
        self.fig, (self.ax_map, self.ax_depth) = plt.subplots(2, 1, figsize=(10, 12))
        self.fig.tight_layout(pad=5.0)
        plt.ion()
        plt.show()

    def extract_data(self, msg):
        x, y, z = [], [], []
        for cone in msg.track: 
            x.append(cone.location.x)
            y.append(cone.location.y)
            z.append(cone.location.z)
        return x, y, z

    def callback(self, t1_msg, t2_msg):
        # Ground Truth fixo da "Pista Certi Bag"
        # X = Largura (Eixo lateral)
        # Z = Profundidade (Distância frontal)
        gt = {
            'width': [0,1.09,0,1.09,0,1.09,0,1.09,0,1.09],
            #'width': [-0.6, 0.6, -0.6, 0.6, -0.6, 0.6, -0.6, 0.6, -0.6, 0.6],
            'depth': [1.88, 1.88, 2.82, 2.82, 3.76, 3.76, 4.70, 4.70, 5.64, 5.64]
            #'depth': [1.95, 1.95, 2.85, 2.85, 3.75, 3.75, 4.65, 4.65, 5.55, 5.55]
        }

        x1, y1, z1 = self.extract_data(t1_msg)
        x2, y2, z2 = self.extract_data(t2_msg)

        self.update_plots(gt, x1, y1, z1, x2, y2, z2)

    def update_plots(self, gt, x1, y1, z1, x2, y2, z2):
        self.ax_map.cla()
        self.ax_depth.cla()
        
        # --- GRÁFICO 1: MAPA ESPACIAL (LARGURA VS PROFUNDIDADE) ---
        # Representação visual de onde os cones estão no plano horizontal
        self.ax_map.scatter(gt['width'], gt['depth'], s=150, edgecolors='g', facecolors='none', label='GT Real', linewidth=2)
        self.ax_map.scatter(x1, z1, c='blue', marker='o', label='Patinho')
        self.ax_map.scatter(x2, z2, c='red', marker='x', label='Luxonis')
        
        self.ax_map.set_title("Mapa Espacial: Largura (X) vs Profundidade (Z)")
        self.ax_map.set_xlabel("Largura (m)")
        self.ax_map.set_ylabel("Profundidade (m)")
        self.ax_map.axis('equal') 
        self.ax_map.legend()
        self.ax_map.grid(True, alpha=0.3)

        # --- GRÁFICO 2: EXCLUSIVO PROFUNDIDADE (Z) ---
        # Comparamos apenas o valor Z de cada detecção contra o ideal
        # Ordenamos os valores detectados para facilitar a visualização da progressão
        z1_sorted = sorted(z1)
        z2_sorted = sorted(z2)
        z_gt_sorted = sorted(gt['depth'])

        self.ax_depth.plot(z_gt_sorted, 'g--', label='Profundidade GT', alpha=0.6, marker='s')
        if z1_sorted:
            self.ax_depth.plot(z1_sorted, 'bo-', label='Profundidade Patinho')
        if z2_sorted:
            self.ax_depth.plot(z2_sorted, 'rx-', label='Profundidade Luxonis')

        self.ax_depth.set_title("Validação Exclusiva de Profundidade (Eixo Z)")
        self.ax_depth.set_ylabel("Distância do Robô (m)")
        self.ax_depth.set_xlabel("ID do Cone (Ordenado por distância)")
        self.ax_depth.legend()
        self.ax_depth.grid(True, linestyle=':', alpha=0.6)

        plt.draw()
        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = DepthValidatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()