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

        # ==========================================
        # Mestre da Realidade (Ground Truth Fixo)
        # ==========================================
        self.full_gt_z = [3.02, 3.02, 6.04, 6.04, 9.06, 9.06, 12.08, 12.08, 15.1, 15.1]
        self.full_labels = [f'Cone {i+1}' for i in range(len(self.full_gt_z))]

        # ==========================================
        # Filtro de Suavização (EMA) para Porcentagem
        # Agora baseado no ÍNDICE da lista, não no valor!
        # ==========================================
        self.alpha_smooth = 0.15 
        self.smoothed_pct_errors = [None for _ in range(len(self.full_gt_z))]

        # ==========================================
        # Configuração das Janelas do Gráfico
        # ==========================================
        plt.ion() 
        
        self.fig_bar, self.ax_bar = plt.subplots(figsize=(10, 6))
        self.fig_bar.canvas.manager.set_window_title('Comparação de Profundidade: Real vs Estimado')
        
        self.fig_err, self.ax_err = plt.subplots(figsize=(10, 6))
        self.fig_err.canvas.manager.set_window_title('Validação de Erro Percentual')
        
        plt.show()

    def callback(self, msg):
        detected_z = [cone.location.z for cone in msg.track]
        
        measured_z_fixed = []
        pct_errors_fixed = []

        # Fazemos uma cópia das detecções deste frame. 
        # Assim que um cone for associado, ele é removido da lista.
        available_detections = detected_z.copy()

        # Usamos enumerate(i) para salvar o erro no índice exato
        for i, gt in enumerate(self.full_gt_z):
            
            # Limite Dinâmico: Aceita 20% de erro da distância real 
            # Garante no mínimo 0.8m para cones muito próximos
            dynamic_threshold = max(0.8, 0.20 * gt)
            
            candidatos = [z for z in available_detections if abs(z - gt) <= dynamic_threshold]

            if candidatos:
                # Encontra a medição do SGBM que mais se aproxima do Ground Truth
                best_meas = min(candidatos, key=lambda z: abs(z - gt))
                measured_z_fixed.append(best_meas)
                
                # Remove da lista para o próximo cone GT não pegar essa mesma leitura
                available_detections.remove(best_meas)

                # ==========================================
                # Cálculo do Erro Percentual
                # ==========================================
                inst_pct_error = (abs(best_meas - gt) / gt) * 100.0
                
                # Aplica suavização EMA usando o índice 'i'
                if self.smoothed_pct_errors[i] is None:
                    self.smoothed_pct_errors[i] = inst_pct_error
                else:
                    self.smoothed_pct_errors[i] = (self.alpha_smooth * inst_pct_error) + ((1 - self.alpha_smooth) * self.smoothed_pct_errors[i])
                
                pct_errors_fixed.append(self.smoothed_pct_errors[i])
            else:
                measured_z_fixed.append(0.0)
                pct_errors_fixed.append(None) 

        self.update_plot(measured_z_fixed, pct_errors_fixed)

    def update_plot(self, measured_z, smoothed_pct_errors):
        self.ax_bar.cla()
        self.ax_err.cla()
        
        x = np.arange(len(self.full_gt_z)) 
        width = 0.35

        # ==========================================
        # JANELA 1: GRÁFICO DE BARRAS (Mantido em Metros)
        # ==========================================
        rects1 = self.ax_bar.bar(x - width/2, self.full_gt_z, width, label='Z Real (GT)', color='forestgreen', alpha=0.7)
        rects2 = self.ax_bar.bar(x + width/2, measured_z, width, label='Z Estimado (Patinho)', color='royalblue')

        self.ax_bar.set_xticks(x)
        self.ax_bar.set_xticklabels(self.full_labels)
        
        teto_grafico = max(self.full_gt_z)
        if max(measured_z) > teto_grafico:
            teto_grafico = max(measured_z)
            
        self.ax_bar.set_ylim(0, max(5.0, teto_grafico * 1.2))

        self.ax_bar.bar_label(rects1, padding=3, fmt='%.2fm')
        labels_azuis = [f'{val:.2f}m' if val > 0 else 'Falhou' for val in measured_z]
        self.ax_bar.bar_label(rects2, labels=labels_azuis, padding=3, color='darkblue')

        self.ax_bar.set_ylabel('Distância (Z) [m]')
        self.ax_bar.set_title('Comparação de Profundidade (Ancorado no GT)')
        self.ax_bar.legend()
        self.ax_bar.grid(axis='y', linestyle=':', alpha=0.5)

        # ==========================================
        # JANELA 2: ERRO PERCENTUAL VS DISTÂNCIA (QUADRÁTICA)
        # ==========================================
        valid_gt = [gt for gt, err in zip(self.full_gt_z, smoothed_pct_errors) if err is not None]
        valid_err = [err for err in smoothed_pct_errors if err is not None]

        if len(valid_gt) > 0:
            self.ax_err.scatter(valid_gt, valid_err, color='red', alpha=0.7, edgecolors='black', s=80, label='Erro Relativo (%)')
            
            # Linha de tendência polinomial de 2º Grau (Parábola)
            if len(valid_gt) > 2:
                z = np.polyfit(valid_gt, valid_err, 2)
                p = np.poly1d(z)
                
                x_linha = np.linspace(min(self.full_gt_z) - 0.5, max(self.full_gt_z) + 0.5, 100)
                y_linha = p(x_linha)
                
                equacao_str = f'Tendência (y = {z[0]:.4f}x² + {z[1]:.2f}x + {z[2]:.2f})'
                
                self.ax_err.plot(x_linha, y_linha, color='purple', linestyle='--', linewidth=2, 
                                 label=equacao_str)

            min_x = min(self.full_gt_z)
            max_x = max(self.full_gt_z)
            
            # Trava o gráfico em um limite razoável para visualização
            max_y = max(valid_err) if valid_err else 5.0
            teto_percentual = max(10.0, max_y * 1.5) 
                
            self.ax_err.set_xlim(left=max(0, min_x - 0.5), right=max_x + 0.5)
            self.ax_err.set_ylim(bottom=-0.5, top=teto_percentual)
            
            from matplotlib.ticker import PercentFormatter
            self.ax_err.yaxis.set_major_formatter(PercentFormatter(decimals=1))
        
        else:
            self.ax_err.set_xlim(0, 16.0)
            self.ax_err.set_ylim(-0.5, 10.0)
            self.ax_err.text(0.5, 0.5, 'Sem dados de erro (Câmera cega)', horizontalalignment='center', verticalalignment='center', transform=self.ax_err.transAxes, fontsize=14, color='gray')

        self.ax_err.axhline(0, color='green', linestyle='-', linewidth=2, label='0% Erro (Ideal)')
        self.ax_err.axhline(5.0, color='orange', linestyle=':', linewidth=2, label='Meta Validação (5%)')

        self.ax_err.set_title('Validação de Erro Relativo (Suavizado)')
        self.ax_err.set_xlabel('Distância Real - GT [m]')
        self.ax_err.set_ylabel('Erro Percentual [%]')
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