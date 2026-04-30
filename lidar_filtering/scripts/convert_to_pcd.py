#!/usr/bin/env python3
"""
ROS2 Node: pcd_saver
Assina /velodyne/points e salva a pointcloud atual em .pcd ao pressionar 'q'.
Baseado na decodificação numpy que já funciona.
"""

import sys
import threading
import termios
import tty
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class PCDSaverNode(Node):
    def __init__(self):
        super().__init__('pcd_saver')

        self.current_cloud: PointCloud2 | None = None
        self.save_count = 0
        self.lock = threading.Lock()

        self.sub = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.cloud_callback,
            10
        )

        self.get_logger().info("pcd_saver iniciado.")
        self.get_logger().info("Pressione 'q' para salvar | ESC/Ctrl+C para sair.")

        self.key_thread = threading.Thread(target=self.key_listener, daemon=True)
        self.key_thread.start()

    def cloud_callback(self, msg: PointCloud2):
        with self.lock:
            self.current_cloud = msg

    def key_listener(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while rclpy.ok():
                ch = sys.stdin.read(1)
                if ch == 'q':
                    self.save_pcd()
                elif ch in ('\x03', '\x1b'):
                    self.get_logger().info("Encerrando...")
                    rclpy.shutdown()
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def save_pcd(self):
        with self.lock:
            cloud = self.current_cloud

        if cloud is None:
            self.get_logger().warn("Nenhuma pointcloud recebida ainda.")
            return

        # DEBUG: conferir estrutura da PointCloud2
        self.get_logger().info(
            f"frame_id={cloud.header.frame_id}, width={cloud.width}, height={cloud.height}, "
            f"point_step={cloud.point_step}, row_step={cloud.row_step}, is_bigendian={cloud.is_bigendian}"
        )

        for field in cloud.fields:
            self.get_logger().info(
                f"{field.name}: offset={field.offset}, datatype={field.datatype}, count={field.count}"
            )

        # Usa a quantidade oficial de pontos da mensagem
        num_points = cloud.width * cloud.height

        if num_points == 0:
            self.get_logger().warn("Pointcloud vazia recebida.")
            return

        # Decodifica os bytes da PointCloud2
        full_data = np.frombuffer(bytes(cloud.data), dtype=np.uint8)

        # Garante que só vai usar os bytes correspondentes aos pontos
        expected_size = num_points * cloud.point_step
        full_data = full_data[:expected_size]

        reshaped = full_data.reshape(num_points, cloud.point_step)

        # X, Y, Z estão normalmente nos primeiros 12 bytes:
        # x -> offset 0
        # y -> offset 4
        # z -> offset 8
        xyz = reshaped[:, 0:12].view(dtype=np.float32).reshape(-1, 3)

        # Remove NaN/Inf
        xyz = xyz[np.all(np.isfinite(xyz), axis=1)]

        if len(xyz) == 0:
            self.get_logger().warn("Todos os pontos eram NaN/Inf, nada salvo.")
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.save_count += 1
        filename = f"pointcloud_{self.save_count}.pcd"

        save_pcd_ascii(filename, xyz)
        self.get_logger().info(f"Salvo: {filename} ({len(xyz)} pontos)")


def save_pcd_ascii(filename: str, points: np.ndarray):
    header = (
        "# .PCD v0.7 - Point Cloud Data\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {len(points)}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\n"
        "DATA ascii\n"
    )
    with open(filename, 'w') as f:
        f.write(header)
        np.savetxt(f, points, fmt="%.6f")


def main(args=None):
    rclpy.init(args=args)
    node = PCDSaverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()