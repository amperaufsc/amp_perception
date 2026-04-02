#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import numpy as np
from fs_msgs.msg import TrackStampedWithCovariance

class DepthBarComparisonNode(Node):
    def __init__(self):
        super().__init__('depth_bar_comparison_node')

        self.subscription = self.create_subscription(
            TrackStampedWithCovariance,
            '/track_pub/patinho',
            self.callback,
            10)

        # Configuração do Gráfico
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        plt.ion()
        plt.show()

        # Ground Truth definido (Pares em 2m, 4m e 5m)
        self.gt_z_values = [2.0, 2.0, 4.0, 4.0, 5.0, 5.0]
        self.labels = ['Par 1 (A)', 'Par 1 (B)', 'Par 2 (A)', 'Par 2 (B)', 'Par 3 (A)', 'Par 3 (B)']

    def callback(self, msg):
        # Extrai e ordena as profundidades detectadas para parear com o GT
        detected_z = sorted([cone.location.z for cone in msg.track])
        
        # Ajusta o tamanho do array detectado para não quebrar o gráfico 
        # (pega apenas os 6 cones mais próximos ou preenche com 0 se faltar)
        display_z = detected_z[:6]
        while len(display_z) < 6:
            display_z.append(0.0)

        self.update_plot(display_z)

    def update_plot(self, measured_z):
        self.ax.cla()
        
        x = np.arange(len(self.labels))  # Localização das labels
        width = 0.35  # Largura das barras

        # Barras de Ground Truth vs Medição
        rects1 = self.ax.bar(x - width/2, self.gt_z_values, width, label='Z Real (GT)', color='forestgreen', alpha=0.7)
        rects2 = self.ax.bar(x + width/2, measured_z, width, label='Z Estimado (Estimated)', color='royalblue')

        # Customização do Gráfico
        self.ax.set_ylabel('Distância (Z) [m]')
        self.ax.set_title('Comparação de Profundidade: Real vs Estimado')
        self.ax.set_xticks(x)
        self.ax.set_xticklabels(self.labels)
        self.ax.set_ylim(0, 7.0)
        self.ax.legend()
        self.ax.grid(axis='y', linestyle=':', alpha=0.5)

        # Adiciona os valores em cima das barras para leitura direta
        self.ax.bar_label(rects1, padding=3, fmt='%.1fm')
        self.ax.bar_label(rects2, padding=3, fmt='%.2fm')

        plt.draw()
        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = DepthBarComparisonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()