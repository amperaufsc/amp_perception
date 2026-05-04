#!/usr/bin/env python3

import os
import sys
import time
import tty
import termios
import threading

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge


class DatasetSaverNode(Node):
    def __init__(self):
        super().__init__('dataset_saver')

        self.bridge = CvBridge()

        self.latest_img_left = None
        self.current_cloud: PointCloud2 | None = None

        self.lock = threading.Lock()
        self.save_count = 0

        self.image_dir = '/home/lucasmoro/imagens'
        self.pcd_dir = '/home/lucasmoro/pcds'

        os.makedirs(self.image_dir, exist_ok=True)
        os.makedirs(self.pcd_dir, exist_ok=True)

        self.sub_img = self.create_subscription(
            Image,
            '/oak/left/image_raw',
            self.image_callback,
            10
        )

        self.sub_cloud = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.cloud_callback,
            10
        )

        self.gui_thread = threading.Thread(target=self.display_loop, daemon=True)
        self.gui_thread.start()

        self.get_logger().info("dataset_saver iniciado.")
        self.get_logger().info("Pressione 'q' para salvar imagem + PCD | 'p'/ESC/Ctrl+C para sair.")

    def image_callback(self, msg: Image):
        img_cv = self.bridge.imgmsg_to_cv2(msg)

        with self.lock:
            self.latest_img_left = img_cv.copy()

    def cloud_callback(self, msg: PointCloud2):
        with self.lock:
            self.current_cloud = msg

    def display_loop(self):
        cv2.namedWindow("imgL", cv2.WINDOW_NORMAL)

        while rclpy.ok():
            with self.lock:
                if self.latest_img_left is None:
                    display_left = None
                else:
                    display_left = self.latest_img_left.copy()

            if display_left is not None:
                img_with_lines = self.draw_grid(display_left.copy())
                cv2.imshow("imgL", img_with_lines)

            key = cv2.waitKey(1) & 0xFF

            if key in (ord('p'), 27):  # p ou ESC
                self.get_logger().info("Encerrando...")
                rclpy.shutdown()
                break

            elif key == ord('q'):
                self.save_pair()

            time.sleep(0.01)

        cv2.destroyAllWindows()

    def save_pair(self):
        with self.lock:
            img = None if self.latest_img_left is None else self.latest_img_left.copy()
            cloud = self.current_cloud

        if img is None:
            self.get_logger().warn("Nenhuma imagem recebida ainda.")
            return

        if cloud is None:
            self.get_logger().warn("Nenhuma pointcloud recebida ainda.")
            return

        self.save_count += 1

        img_filename = os.path.join(
            self.image_dir,
            f"save{self.save_count:04d}.jpg"
        )

        pcd_filename = os.path.join(
            self.pcd_dir,
            f"save{self.save_count:04d}.pcd"
        )

        cv2.imwrite(img_filename, img)

        xyz = self.pointcloud2_to_xyz(cloud)

        if xyz is None or len(xyz) == 0:
            self.get_logger().warn("Pointcloud inválida/vazia. Imagem foi salva, mas PCD não.")
            return

        save_pcd_ascii(pcd_filename, xyz)

        self.get_logger().info(
            f"Salvo par {self.save_count:04d}: "
            f"{img_filename} + {pcd_filename} ({len(xyz)} pontos)"
        )

    def pointcloud2_to_xyz(self, cloud: PointCloud2):
        self.get_logger().info(
            f"frame_id={cloud.header.frame_id}, width={cloud.width}, height={cloud.height}, "
            f"point_step={cloud.point_step}, row_step={cloud.row_step}, is_bigendian={cloud.is_bigendian}"
        )

        for field in cloud.fields:
            self.get_logger().info(
                f"{field.name}: offset={field.offset}, datatype={field.datatype}, count={field.count}"
            )

        num_points = cloud.width * cloud.height

        if num_points == 0:
            return None

        full_data = np.frombuffer(bytes(cloud.data), dtype=np.uint8)

        expected_size = num_points * cloud.point_step
        full_data = full_data[:expected_size]

        reshaped = full_data.reshape(num_points, cloud.point_step)

        # Pelo teu print:
        # x -> offset 0, float32
        # y -> offset 4, float32
        # z -> offset 8, float32
        xyz = reshaped[:, 0:12].view(dtype=np.float32).reshape(-1, 3)

        xyz = xyz[np.all(np.isfinite(xyz), axis=1)]

        return xyz

    def draw_grid(self, img):
        n_lines = 20

        h, w = img.shape[:2]
        step = h // n_lines

        for y in range(0, h, step):
            cv2.line(img, (0, y), (w, y), (0, 255, 0), 1)

        for x in range(0, w, step):
            cv2.line(img, (x, 0), (x, h), (0, 255, 0), 1)

        return img


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

    node = DatasetSaverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()