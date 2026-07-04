#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer
from fs_msgs.msg import Track, TrackStamped, TrackStampedWithCovariance
import matplotlib.pyplot as plt


class LidarTrackPlot(Node):
    def __init__(self):
        super().__init__('lidar_track_plot')

        self.track_lidar = Subscriber(self, TrackStamped, "/track_lidar")
        self.track_camera = Subscriber(self, TrackStampedWithCovariance, "/track")

        self.time_sync = ApproximateTimeSynchronizer(
            [self.track_camera, self.track_lidar],
            queue_size=10,
            slop=1.0
        )
        self.time_sync.registerCallback(self.sync_callback)

        # guarda só números (x,z)
        self.lidar_xz = []
        self.camera_xz = []

    def sync_callback(self, track_camera, track_lidar):
        self.lidar_xz = [(c.location.x, c.location.z) for c in track_lidar.track]
        self.camera_xz = [(c.location.x, c.location.z) for c in track_camera.track]
        
        self.get_logger().info(f"recebendo: cam={len(self.camera_xz)} lidar={len(self.lidar_xz)}")


def main(args=None):
    rclpy.init(args=args)
    node = LidarTrackPlot()

    lidar_xz = []
    cam_xz = []

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # captura último estado antes de destruir
        lidar_xz = list(node.lidar_xz)
        cam_xz = list(node.camera_xz)

        try:
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()

    # --- pontos manuais em (x,z) ---
    manual_xz = [
        (1, 2),
        (4, 5),
        (2, 8),
        (7, 3),
        (5, 5),
    ]

    man_x = [x for x, z in manual_xz]
    man_z = [z for x, z in manual_xz]

    cam_x = [x for x, z in cam_xz]
    cam_z = [z for x, z in cam_xz]

    lid_x = [x for x, z in lidar_xz]
    lid_z = [z for x, z in lidar_xz]

    # --- plot X vs Z ---
    plt.figure()
    plt.scatter(man_x, man_z, label="manual (x,z)", marker="x")
    plt.scatter(cam_x, cam_z, label="camera (x,z)", marker="o")
    plt.scatter(lid_x, lid_z, label="lidar (x,z)", marker="^")

    plt.xlabel("X")
    plt.ylabel("Z")
    plt.title("Tracks 2D (X vs Z) - último estado recebido")
    plt.grid(True)
    plt.legend()

    # limites automáticos (inclui negativos)
    all_x = man_x + cam_x + lid_x
    all_z = man_z + cam_z + lid_z
    if all_x and all_z:
        min_x, max_x = min(all_x), max(all_x)
        min_z, max_z = min(all_z), max(all_z)
        pad_x = (max_x - min_x) * 0.2 if max_x != min_x else 1.0
        pad_z = (max_z - min_z) * 0.2 if max_z != min_z else 1.0
        plt.xlim(min_x - pad_x, max_x + pad_x)
        plt.ylim(min_z - pad_z, max_z + pad_z)

    plt.axis('equal')
    plt.show()


if __name__ == '__main__':
    main()