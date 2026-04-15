#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer
import matplotlib.pyplot as plt
import numpy as np
from fs_msgs.msg import TrackStampedWithCovariance

class DepthValidatorNode(Node):
    def __init__(self):
        super().__init__('depth_validator_node')

        # Subscribers
        self.sub_t1 = Subscriber(self, TrackStampedWithCovariance, '/track_pub/patinho')
        self.sub_t2 = Subscriber(self, TrackStampedWithCovariance, '/track_pub/luxonis')

        # Sincronização (slop de 1.0s costuma ser suficiente para evitar perdas)
        self.ts = ApproximateTimeSynchronizer([self.sub_t1, self.sub_t2], 10, 1.0)
        self.ts.registerCallback(self.callback)

        # Configuração: Mapa Espacial e Gráfico de Erro Residual
        self.fig, (self.ax_map, self.ax_error) = plt.subplots(2, 1, figsize=(10, 12))
        self.fig.tight_layout(pad=5.0)
        plt.ion()
        plt.show()

    def extract_data(self, msg):
        # Extraímos como array numpy para facilitar cálculos matemáticos
        coords = []
        for cone in msg.track: 
            coords.append([cone.location.x, cone.location.z])
        return np.array(coords) if coords else np.empty((0, 2))

    def calculate_depth_errors(self, gt_points, detected_points):
        """
        Para cada cone do GT, encontra o detectado mais próximo e calcula o erro de Z.
        Retorna (Distância_GT, Erro_Z)
        """
        gt_depths = []
        errors = []

        for gt in gt_points:
            if detected_points.shape[0] == 0:
                break
            
            # Calcula distância euclidiana (X e Z) para parear o cone
            dists = np.linalg.norm(detected_points - gt, axis=1)
            idx_min = np.argmin(dists)
            
            # Só pareia se o cone estiver a menos de 1.2m (evita parear com cones errados)
            if dists[idx_min] < 1.2:
                z_gt = gt[1]
                z_detected = detected_points[idx_min][1]
                
                gt_depths.append(z_gt)
                errors.append(z_detected - z_gt) # Erro Positivo = Sensor viu mais longe
        
        return gt_depths, errors

    def callback(self, t1_msg, t2_msg):
        # Ground Truth fixo da "Pista Certi Bag" (X, Z)
        gt_points = np.array([
            [0.0, 2], [0.7, 2]
        ])

        t1_data = self.extract_data(t1_msg)
        t2_data = self.extract_data(t2_msg)

        # Cálculo dos erros baseados na distância GT
        dist_gt1, err1 = self.calculate_depth_errors(gt_points, t1_data)
        dist_gt2, err2 = self.calculate_depth_errors(gt_points, t2_data)

        self.update_plots(gt_points, t1_data, t2_data, dist_gt1, err1, dist_gt2, err2)

    def update_plots(self, gt_points, t1, t2, d1, e1, d2, e2):
        self.ax_map.cla()
        self.ax_error.cla()
        
        # --- GRÁFICO 1: MAPA ESPACIAL (LARGURA VS PROFUNDIDADE) ---
        self.ax_map.scatter(gt_points[:,0], gt_points[:,1], s=150, edgecolors='g', facecolors='none', label='GT Real', linewidth=2)
        if t1.size > 0: self.ax_map.scatter(t1[:,0], t1[:,1], c='blue', label='Patinho')
        if t2.size > 0: self.ax_map.scatter(t2[:,0], t2[:,1], c='red', marker='x', label='Luxonis')
        
        self.ax_map.set_title("Mapa 2D: Largura (X) vs Profundidade (Z)")
        self.ax_map.set_xlabel("Largura (m)")
        self.ax_map.set_ylabel("Profundidade (m)")
        self.ax_map.axis('equal') 
        self.ax_map.legend()
        self.ax_map.grid(True, alpha=0.3)

        # --- GRÁFICO 2: ERRO DE PROFUNDIDADE EM FUNÇÃO DA DISTÂNCIA ---
        self.ax_error.axhline(y=0, color='black', linestyle='-', alpha=0.5) # Linha de erro zero
        
        if d1:
            self.ax_error.scatter(d1, e1, c='blue', label='Erro Patinho', s=60)
            # Linha de tendência para Patinho
            coeffs = np.polyfit(d1, e1, 1)
            poly = np.poly1d(coeffs)
            self.ax_error.plot(d1, poly(d1), "b--", alpha=0.4)

        if d2:
            self.ax_error.scatter(d2, e2, c='red', marker='x', label='Erro Luxonis', s=60)
            # Linha de tendência para Luxonis
            coeffs = np.polyfit(d2, e2, 1)
            poly = np.poly1d(coeffs)
            self.ax_error.plot(d2, poly(d2), "r--", alpha=0.4)

        self.ax_error.set_title("Erro de Medição Z vs. Distância Real")
        self.ax_error.set_ylabel("Erro Residual (Detectado - GT) [m]")
        self.ax_error.set_xlabel("Distância Real (GT Z) [m]")
        self.ax_error.set_ylim(-0.8, 0.8) # Ajuste o zoom do erro aqui
        self.ax_error.legend()
        self.ax_error.grid(True, linestyle=':', alpha=0.6)

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