#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import matplotlib.pyplot as plt

from fs_msgs.msg import TrackStamped


class DepthMapNode(Node):
    def __init__(self):
        super().__init__("depth_map_node")

        self.subscription = self.create_subscription(
            TrackStamped,
            "/track_lidar",
            self.callback,
            10,
        )

        # ==========================================
        # Ground Truth de profundidade (eixo Y fixo)
        # Cones fixos no eixo X: 0 (esquerda) e 1 (direita)
        # ==========================================
        self.gt_points = [
            [0, 3.02], [1, 3.02],
            [0, 4.53], [1, 4.53],
            [0, 6.04], [1, 6.04],
            [0, 7.55], [1, 7.55],
            [0, 9.06], [1, 9.06],
            [0, 10.57], [1, 10.57],
        ]
        self.i = 1
        self.pinto = 0
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(6, 10))
        self.fig.canvas.manager.set_window_title("Mapa 2D de Cones")
        plt.show()

    def callback(self, msg):
        ros_points = [
            [i % 2, cone.location.x]   # X fixo: 0 ou 1 alternando, Y = profundidade
            for i, cone in enumerate(msg.track)
        ]

        self.update_plot(ros_points)

    def update_plot(self, ros_points):
        self.ax.cla()

        # Ground Truth
        gt_x = [p[0] for p in self.gt_points]
        gt_y = [p[1] for p in self.gt_points]

        self.ax.scatter(gt_x, gt_y, color="green", s=100, label="Ground Truth", zorder=3)

        # Linha conectando pares GT (esquerda-direita por profundidade)
        for i in range(0, len(self.gt_points), 2):
            self.ax.plot(
                [self.gt_points[i][0], self.gt_points[i + 1][0]],
                [self.gt_points[i][1], self.gt_points[i + 1][1]],
                color="green",
                linestyle="--",
                alpha=0.4,
            )

        # Track ROS
        if ros_points:
            ros_x = [p[0] for p in ros_points]
            ros_y = [p[1] for p in ros_points]

            self.ax.scatter(ros_x, ros_y, color="blue", s=80, label="Track LiDAR", zorder=3)

            for i, p in enumerate(ros_points):
                self.ax.text(p[0] + 0.05, p[1], str(i), fontsize=9, color="blue")
        else:
            self.ax.text(
                0.5, 0.5,
                "Nenhum cone recebido",
                ha="center", va="center",
                transform=self.ax.transAxes,
                fontsize=13, color="gray",
            )

        self.ax.set_xlabel("Lateral [m]")
        self.ax.set_ylabel("Profundidade [m]")
        self.ax.set_title("Mapa 2D de Cones")
        self.ax.legend(loc="upper right")
        self.ax.grid(True, linestyle=":", alpha=0.6)
        self.ax.set_aspect("equal", adjustable="box")

        self.ax.set_xlim(-1.5, 2.5)   # fixo — mesmo para GT e track
        self.ax.set_ylim(min(gt_y) - 1.0, max(gt_y) + 1.0)  # Y baseado só no GT

        self.fig.canvas.draw_idle()
        plt.pause(0.01)


def main(args=None):
    rclpy.init(args=args)
    node = DepthMapNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()