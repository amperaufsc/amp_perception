#!/usr/bin/env python3

from __future__ import print_function

import numpy as np
import cv2
import os

import rclpy
from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from amp_perception.perception.utils.perception_calc import PerceptionProcess
import yaml
import time
import threading

bridge = CvBridge()


class NoDisparidadeShowWindow(Node):

    def __init__(self):
        super().__init__('disp_map_show')
                
        self.img_L = Subscriber(self, Image, "/oak/left/image_raw")
        self.img_R = Subscriber(self, Image, "/oak/right/image_raw")
        
        max_delay = 0.05
        self.time_sync = ApproximateTimeSynchronizer([self.img_L, self.img_R],10,max_delay)
        self.time_sync.registerCallback(self.sync_callback)
                
        self.lock = threading.Lock()
        self.gui_thread = threading.Thread(target=self.display_loop)
        self.gui_thread.start()
        
        self.get_logger().info("init finalizado")
        

    def sync_callback(self, img_L_raw, img_R_raw):

        img_L_cv = bridge.imgmsg_to_cv2(img_L_raw)
        img_R_cv = bridge.imgmsg_to_cv2(img_R_raw)

        with self.lock:
            self.latest_img_left = img_L_cv.copy()
            self.latest_img_right = img_R_cv.copy()

    def display_loop(self):
        cv2.namedWindow("Camera Images", cv2.WINDOW_NORMAL)
        
        # 1. Configura a pasta e garante que ela existe antes de começar
        save_dir = '/home/otaviogoulart/Desktop/Turno/dataset_calib'
        os.makedirs(save_dir, exist_ok=True)
        
        # 2. Inicializa o contador de pares de fotos
        image_counter = 0

        while rclpy.ok():
            with self.lock:
                display_left = self.latest_img_left
                display_right = self.latest_img_right
                
            if display_left is not None:

                img_with_lines = self.draw_epilines(display_left.copy())
                cv2.imshow("imgL", img_with_lines)
                cv2.imshow("imgR", display_right)

            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break
            elif key == ord('b'):
                # 3. Formata os nomes com 4 dígitos (ex: img_left_0000.jpg)
                left_filename = os.path.join(save_dir, f'img_left_{image_counter:04d}.jpg')
                right_filename = os.path.join(save_dir, f'img_right_{image_counter:04d}.jpg')
                
                # 4. Salva as imagens e avisa no terminal
                cv2.imwrite(left_filename, self.latest_img_left)
                cv2.imwrite(right_filename, display_right)
                print(f"[Dataset] Par de fotos {image_counter:04d} salvo com sucesso!")
                
                # 5. Incrementa o contador para não sobrescrever na próxima vez
                image_counter += 1

            time.sleep(0.01)
            
        cv2.destroyAllWindows()

    def draw_epilines(self, imgL):
        n_lines = 20
        # Assume-se que 'bridge' já foi instanciado (CvBridge)

        h, w = imgL.shape[:2]
        
        # Calcula o tamanho do quadrado para manter a proporção da grade
        step = h // n_lines
        
        # 1. Desenha as linhas horizontais (Verifica o alinhamento epipolar)
        for y in range(0, h, step):
            cv2.line(imgL, (0, y), (w * 2, y), (0, 255, 0), 1)

        # 2. Desenha as linhas verticais (Verifica o campo de visão e escala)
        for x in range(0, w * 2, step):
            cv2.line(imgL, (x, 0), (x, h), (0, 255, 0), 1)

        return imgL

def main(args=None):
    rclpy.init(args=args)
    
    disp_map_show = NoDisparidadeShowWindow()
    
    rclpy.spin(disp_map_show)
    disp_map_show.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    cv2.destroyAllWindows()
    