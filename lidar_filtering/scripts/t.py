#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import PercentFormatter

from fs_msgs.msg import TrackStamped


class DepthBarComparisonNode(Node):
    def __init__(self):
        super().__init__("depth_bar_comparison_node")

        self.subscription = self.create_subscription(
            TrackStamped,
            "/track_lidar",
            self.callback,
            10,
        )

        # ==========================================
        # Ground Truth de profundidade
        # ==========================================
        self.full_gt_depth = [3.02, 4.53, 6.04, 7.55, 9.06, 10.57]
        self.full_labels = [f"Cone {i + 1}" for i in range(len(self.full_gt_depth))]

        self.association_threshold = 0.8

        # ==========================================
        # Ground Truth no plano do gráfico
        #
        # Aqui:
        # X = lateral
        # Y = profundidade
        # ==========================================
        self.gt_points = [
            [0, 3.02], [1, 3.02],
            [0, 4.53], [1, 4.53],
            [0, 6.04], [1, 6.04],
            [0, 7.55], [1, 7.55],
            [0, 9.06], [1, 9.06],
            [0, 10.57], [1, 10.57],
        ]

        # ==========================================
        # Filtro de suavização EMA
        # ==========================================
        self.alpha_smooth = 0.15
        self.smoothed_pct_errors = {gt: None for gt in self.full_gt_depth}

        # ==========================================
        # Configuração das janelas
        # ==========================================
        plt.ion()

        self.fig_bar, self.ax_bar = plt.subplots(figsize=(10, 6))
        self.fig_bar.canvas.manager.set_window_title(
            "Comparação de Profundidade: Real vs Estimado"
        )

        self.fig_err, self.ax_err = plt.subplots(figsize=(10, 6))
        self.fig_err.canvas.manager.set_window_title(
            "Validação de Erro Percentual"
        )

        self.fig_xy, self.ax_xy = plt.subplots(figsize=(8, 8))
        self.fig_xy.canvas.manager.set_window_title(
            "Comparação XY: Ground Truth vs Track ROS"
        )

        plt.show()

    def callback(self, msg):
        # ==========================================
        # IMPORTANTE:
        #
        # No teu clusterize:
        # cone.location.x = p_cam(2)   -> profundidade
        # cone.location.y = -p_cam(0)  -> lateral
        # cone.location.z = 0
        #
        # Então:
        # profundidade = cone.location.x
        # lateral      = cone.location.y
        # ==========================================

        detected_depth = [cone.location.x for cone in msg.track]

        ros_points = [
            [cone.location.y, cone.location.x]
            for cone in msg.track
        ]

        measured_depth_fixed = []
        pct_errors_fixed = []

        # ==========================================
        # Associação ancorada no Ground Truth
        # ==========================================
        for gt in self.full_gt_depth:
            candidatos = [
                depth for depth in detected_depth
                if abs(depth - gt) <= self.association_threshold
            ]

            if candidatos:
                best_meas = min(candidatos, key=lambda depth: abs(depth - gt))
                measured_depth_fixed.append(best_meas)

                inst_pct_error = (abs(best_meas - gt) / gt) * 100.0

                if self.smoothed_pct_errors[gt] is None:
                    self.smoothed_pct_errors[gt] = inst_pct_error
                else:
                    self.smoothed_pct_errors[gt] = (
                        self.alpha_smooth * inst_pct_error
                        + (1 - self.alpha_smooth) * self.smoothed_pct_errors[gt]
                    )

                pct_errors_fixed.append(self.smoothed_pct_errors[gt])

            else:
                measured_depth_fixed.append(0.0)
                pct_errors_fixed.append(None)

        self.update_plot(measured_depth_fixed, pct_errors_fixed, ros_points)

    def update_plot(self, measured_depth, smoothed_pct_errors, ros_points):
        self.ax_bar.cla()
        self.ax_err.cla()
        self.ax_xy.cla()

        self.update_depth_bar_plot(measured_depth)
        self.update_error_plot(smoothed_pct_errors)
        self.update_xy_plot(ros_points)

        self.fig_bar.canvas.draw_idle()
        self.fig_err.canvas.draw_idle()
        self.fig_xy.canvas.draw_idle()

        plt.pause(0.01)

    def update_depth_bar_plot(self, measured_depth):
        x = np.arange(len(self.full_gt_depth))
        width = 0.35

        rects1 = self.ax_bar.bar(
            x - width / 2,
            self.full_gt_depth,
            width,
            label="Profundidade Real (GT)",
            color="forestgreen",
            alpha=0.7,
        )

        rects2 = self.ax_bar.bar(
            x + width / 2,
            measured_depth,
            width,
            label="Profundidade Estimada",
            color="royalblue",
        )

        self.ax_bar.set_xticks(x)
        self.ax_bar.set_xticklabels(self.full_labels)

        teto_grafico = max(self.full_gt_depth)

        if measured_depth and max(measured_depth) > teto_grafico:
            teto_grafico = max(measured_depth)

        self.ax_bar.set_ylim(0, max(5.0, teto_grafico * 1.2))

        self.ax_bar.bar_label(rects1, padding=3, fmt="%.2fm")

        labels_azuis = [
            f"{val:.2f}m" if val > 0 else "Falhou"
            for val in measured_depth
        ]

        self.ax_bar.bar_label(
            rects2,
            labels=labels_azuis,
            padding=3,
            color="darkblue",
        )

        self.ax_bar.set_ylabel("Profundidade [m]")
        self.ax_bar.set_title("Comparação de Profundidade")
        self.ax_bar.legend()
        self.ax_bar.grid(axis="y", linestyle=":", alpha=0.5)

    def update_error_plot(self, smoothed_pct_errors):
        valid_gt = [
            gt for gt, err in zip(self.full_gt_depth, smoothed_pct_errors)
            if err is not None
        ]

        valid_err = [
            err for err in smoothed_pct_errors
            if err is not None
        ]

        if len(valid_gt) > 0:
            self.ax_err.scatter(
                valid_gt,
                valid_err,
                color="red",
                alpha=0.7,
                edgecolors="black",
                s=80,
                label="Erro Relativo (%)",
            )

            if len(valid_gt) > 1:
                z = np.polyfit(valid_gt, valid_err, 1)
                p = np.poly1d(z)

                x_linha = np.linspace(
                    min(self.full_gt_depth) - 0.5,
                    max(self.full_gt_depth) + 0.5,
                    100,
                )

                y_linha = p(x_linha)

                self.ax_err.plot(
                    x_linha,
                    y_linha,
                    color="purple",
                    linestyle="--",
                    linewidth=2,
                    label=f"Tendência (y = {z[0]:.2f}x + {z[1]:.2f})",
                )

            min_x = min(self.full_gt_depth)
            max_x = max(self.full_gt_depth)

            max_y = max(valid_err) if valid_err else 5.0
            teto_percentual = max(10.0, max_y * 1.5)

            self.ax_err.set_xlim(left=max(0, min_x - 0.5), right=max_x + 0.5)
            self.ax_err.set_ylim(bottom=-0.5, top=teto_percentual)

            self.ax_err.yaxis.set_major_formatter(PercentFormatter(decimals=1))

        else:
            self.ax_err.set_xlim(0, 8.0)
            self.ax_err.set_ylim(-0.5, 10.0)
            self.ax_err.text(
                0.5,
                0.5,
                "Sem dados de erro",
                horizontalalignment="center",
                verticalalignment="center",
                transform=self.ax_err.transAxes,
                fontsize=14,
                color="gray",
            )

        self.ax_err.axhline(
            0,
            color="green",
            linestyle="-",
            linewidth=2,
            label="0% Erro",
        )

        self.ax_err.axhline(
            5.0,
            color="orange",
            linestyle=":",
            linewidth=2,
            label="Meta 5%",
        )

        self.ax_err.set_title("Erro Relativo Suavizado")
        self.ax_err.set_xlabel("Profundidade Real - GT [m]")
        self.ax_err.set_ylabel("Erro Percentual [%]")
        self.ax_err.grid(True, linestyle=":", alpha=0.6)
        self.ax_err.legend(loc="upper left")

    def update_xy_plot(self, ros_points):
        # ==========================================
        # Ground Truth
        # X = lateral
        # Y = profundidade
        # ==========================================
        gt_x = [p[0] for p in self.gt_points]
        gt_y = [p[1] for p in self.gt_points]

        self.ax_xy.scatter(
            gt_x,
            gt_y,
            color="green",
            s=80,
            label="Ground Truth",
        )

        for i in range(0, len(self.gt_points), 2):
            x_par = [self.gt_points[i][0], self.gt_points[i + 1][0]]
            y_par = [self.gt_points[i][1], self.gt_points[i + 1][1]]

            self.ax_xy.plot(
                x_par,
                y_par,
                color="green",
                linestyle="--",
                alpha=0.5,
            )

        # ==========================================
        # Track ROS já convertida:
        #
        # ros_x = cone.location.y  -> lateral
        # ros_y = cone.location.x  -> profundidade
        # ==========================================
        if ros_points:
            ros_x = [p[0] for p in ros_points]
            ros_y = [p[1] for p in ros_points]

            self.ax_xy.scatter(
                ros_x,
                ros_y,
                color="blue",
                s=60,
                label="Track ROS",
            )

            for i, point in enumerate(ros_points):
                self.ax_xy.text(
                    point[0],
                    point[1],
                    f" {i}",
                    fontsize=9,
                    color="blue",
                )

        else:
            self.ax_xy.text(
                0.5,
                0.5,
                "Nenhum cone recebido na track",
                horizontalalignment="center",
                verticalalignment="center",
                transform=self.ax_xy.transAxes,
                fontsize=14,
                color="gray",
            )

        self.ax_xy.set_title("Comparação: Ground Truth vs Track ROS")
        self.ax_xy.set_xlabel("Lateral [m]")
        self.ax_xy.set_ylabel("Profundidade [m]")

        self.ax_xy.grid(True, linestyle=":", alpha=0.6)
        self.ax_xy.legend(loc="upper right")
        self.ax_xy.set_aspect("equal", adjustable="box")

        all_x = gt_x.copy()
        all_y = gt_y.copy()

        if ros_points:
            all_x += [p[0] for p in ros_points]
            all_y += [p[1] for p in ros_points]

        margin = 1.0

        self.ax_xy.set_xlim(min(all_x) - margin, max(all_x) + margin)
        self.ax_xy.set_ylim(min(all_y) - margin, max(all_y) + margin)


def main(args=None):
    rclpy.init(args=args)

    node = DepthBarComparisonNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()