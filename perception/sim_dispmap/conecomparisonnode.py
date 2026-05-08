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

        # Ground Truth definido (Lista completa)
        self.full_gt_z = [1.51, 3.02, 4.53, 6.04, 7.55, 9.06, 10.57, 12.08]

        # Nomenclatura base
        self.full_labels = [f'Cone {i+1}' for i in range(len(self.full_gt_z))]

        # ==========================================
        # Configuração das Janelas do Gráfico
        # ==========================================
        plt.ion() 
        
        self.fig_bar, self.ax_bar = plt.subplots(figsize=(10, 6))
        self.fig_bar.canvas.manager.set_window_title('Comparação de Profundidade: Real vs Estimado')
        
        self.fig_err, self.ax_err = plt.subplots(figsize=(10, 6))
        self.fig_err.canvas.manager.set_window_title('Análise de Erro por Distância')
        
        plt.show()

    def callback(self, msg):
        # 1. Extrai as profundidades detectadas (não precisa ordenar ainda)
        detected_z = [cone.location.z for cone in msg.track]
        
        active_meas = []
        active_gt = []
        active_labels = []

        # 2. Pareamento Inteligente (Data Association)
        # Para cada cone que a câmera viu, procura qual é o Ground Truth mais perto dele
        for z_meas in detected_z:
            # Calcula a distância absoluta deste cone para todos os GTs possíveis
            diferencas = np.abs(np.array(self.full_gt_z) - z_meas)
            
            # Pega o índice do GT que tem a menor diferença
            idx_closest = np.argmin(diferencas)
            gt_correspondente = self.full_gt_z[idx_closest]
            
            # Adiciona aos pares ativos
            active_meas.append(z_meas)
            active_gt.append(gt_correspondente)
            
            # Descobre o nome do cone (Cone 1, Cone 2, etc.) baseado na posição original do GT
            active_labels.append(self.full_labels[idx_closest])

        # 3. Ordenação Simultânea (Para o gráfico de barras não ficar embaralhado)
        if len(active_meas) > 0:
            # Junta o GT, o Medido e o Label, ordena pelo valor do GT, e separa de volta
            pares_ordenados = sorted(zip(active_gt, active_meas, active_labels))
            active_gt, active_meas, active_labels = zip(*pares_ordenados)
            
            # Converte de volta para lista
            active_gt = list(active_gt)
            active_meas = list(active_meas)
            active_labels = list(active_labels)

        # Envia apenas os dados pareados e ordenados para o plot
        self.update_plot(active_meas, active_gt, active_labels)

    def update_plot(self, measured_z, gt_z, labels):
        self.ax_bar.cla()
        self.ax_err.cla()
        
        num_cones = len(measured_z)

        # ==========================================
        # JANELA 1: GRÁFICO DE BARRAS (Somente cones detectados)
        # ==========================================
        if num_cones > 0:
            x = np.arange(num_cones)
            width = 0.35

            rects1 = self.ax_bar.bar(x - width/2, gt_z, width, label='Z Real (GT)', color='forestgreen', alpha=0.7)
            rects2 = self.ax_bar.bar(x + width/2, measured_z, width, label='Z Estimado (Patinho)', color='royalblue')

            self.ax_bar.set_xticks(x)
            self.ax_bar.set_xticklabels(labels)
            
            # Zoom Dinâmico
            teto_grafico = max(max(gt_z), max(measured_z))
            self.ax_bar.set_ylim(0, max(5.0, teto_grafico * 1.2))

            self.ax_bar.bar_label(rects1, padding=3, fmt='%.2fm')
            self.ax_bar.bar_label(rects2, padding=3, fmt='%.2fm')
        else:
            # Mensagem caso o carro não veja nada
            self.ax_bar.text(0.5, 0.5, 'Nenhum Cone Detectado', horizontalalignment='center', verticalalignment='center', transform=self.ax_bar.transAxes, fontsize=14, color='gray')
            self.ax_bar.set_ylim(0, 5.0)

        self.ax_bar.set_ylabel('Distância (Z) [m]')
        self.ax_bar.set_title('Comparação de Profundidade: Real vs Estimado')
        self.ax_bar.legend()
        self.ax_bar.grid(axis='y', linestyle=':', alpha=0.5)


        # ==========================================
        # JANELA 2: ERRO VS DISTÂNCIA
        # ==========================================
        if num_cones > 0:
            # Como já garantimos que medido e GT têm o mesmo tamanho e são apenas cones válidos, o erro é direto:
            erros_abs = np.abs(np.array(measured_z) - np.array(gt_z))

            self.ax_err.scatter(gt_z, erros_abs, color='red', alpha=0.7, edgecolors='black', s=80, label='Erro (Cone)')
            
            # Linha de tendência quadrática (exige no mínimo 3 pontos para formar a parábola)
            if num_cones > 2:
                a, b, c = np.polyfit(gt_z, erros_abs, 2)
                x_linha = np.linspace(min(self.full_gt_z) - 0.5, max(self.full_gt_z) + 0.5, 100)
                y_linha = a * (x_linha**2) + b * x_linha + c
                self.ax_err.plot(x_linha, y_linha, color='purple', linestyle='--', linewidth=2, 
                                 label=f'Tendência (y = {a:.3f}x² + {b:.3f}x + {c:.3f})')

            # Zoom Dinâmico
            min_x = min(gt_z)
            max_x = max(gt_z)
            max_y = max(erros_abs)
            
            margem_x = (max_x - min_x) * 0.15
            if margem_x == 0: margem_x = 1.0 
                
            self.ax_err.set_xlim(left=max(0, min_x - margem_x), right=max_x + margem_x)
            self.ax_err.set_ylim(bottom=-0.05, top=max(0.1, max_y * 1.3))
        
        else:
            self.ax_err.set_xlim(0, 10.0)
            self.ax_err.set_ylim(-0.05, 1.0)

        self.ax_err.axhline(0, color='green', linestyle='-', linewidth=2, label='Erro Zero (Ideal)')
        self.ax_err.set_title('Comportamento do Erro de Profundidade')
        self.ax_err.set_xlabel('Distância Real - GT [m]')
        self.ax_err.set_ylabel('Erro Absoluto |Track - GT| [m]')
        self.ax_err.grid(True, linestyle=':', alpha=0.6)
        self.ax_err.legend(loc='upper left')

        # ==========================================
        # Renderização Sincronizada
        # ==========================================
        self.fig_bar.canvas.draw_idle()
        self.fig_err.canvas.draw_idle()
        
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