#!/usr/bin/env python3

from __future__ import print_function

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from std_msgs.msg import Header
from message_filters import Subscriber, ApproximateTimeSynchronizer
from fs_msgs.msg import TrackStampedWithCovariance, TrackStamped
from yolov8_msgs.msg import Yolov8Inference
import yaml
import time
import threading

bridge = CvBridge()


class NoDisparidadeShowWindow(Node):

    def __init__(self):
        super().__init__('disp_map_show')
        self.get_logger().info("DisparityImage foi iniciallizado")
                
        self.image_left_sub = Subscriber(self, Image, "/fsds/cam2/image_color")
        self.image_right_sub = Subscriber(self, Image, "/fsds/cam1/image_color")
        self.yolo_sub = Subscriber(self, Yolov8Inference, "/inferenceresult")
        
        max_delay = 0.05
        self.time_sync = ApproximateTimeSynchronizer([self.image_left_sub,self.image_right_sub, self.yolo_sub],10,max_delay)
        self.time_sync.registerCallback(self.sync_callback)
        
        #Criação de uma thread dedicada a atualizar GUI, para que a atualização aconteça paralelamente ao processamento das imagens e, assim, não uma função "independe" da outra
        
        self.latest_disparity_map = None
        self.lock = threading.Lock()
        self.gui_thread = threading.Thread(target=self.display_loop)
        self.gui_thread.start()
        
        self.get_logger().info("init finalizado")
        

    def sync_callback(self, imgL, imgR, yolo):
        
        imgL = bridge.imgmsg_to_cv2(imgL, desired_encoding='bgr8')
        imgR = bridge.imgmsg_to_cv2(imgR, desired_encoding='bgr8')
        
        disp = self.DisparityProcess(imgL, imgR)
        disp = self.draw_yolo_boxes(disp, yolo)
        Img_L_yolo = self.draw_yolo_boxes(imgL,yolo)
        
        with self.lock:
            #self.latest_disparity_map =  disp_yolo.copy()   
            self.latest_disparity_map = disp.copy()  
            self.imgL_cv2 = Img_L_yolo.copy()
        
    def  DisparityProcess(self, imgL, imgR):
        
        self.get_logger().info("Imagem está sendo processada")

        imgL = cv2.cvtColor(imgL,cv2.COLOR_BGR2GRAY)
        imgR = cv2.cvtColor(imgR,cv2.COLOR_BGR2GRAY)

        # Creating an object of StereoSGBM algorithm
        stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=16*11,
            blockSize=7,
            P1=8*3*7**2,
            P2=32*3*7**2,
            disp12MaxDiff=12,
            uniquenessRatio=3,
            speckleWindowSize=100,
            speckleRange=64,
            preFilterCap=63,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )
        # Calculating disparith using the StereoSGBM algorithm
        disp_map = stereo.compute(imgL, imgR).astype(np.float32) / 16

        # Calculating disparith using the StereoSGBM algorithm

        disp_map = cv2.normalize(disp_map,None, 0, 255, cv2.NORM_MINMAX)
        disp_map = np.uint8(disp_map)
        disp_map[disp_map < 0] = 0
        disp_map[disp_map > 64] = 64
        disp_vis = (disp_map / np.max(disp_map) * 255).astype(np.uint8)
        disp_vis = cv2.medianBlur(disp_vis, 5)  

        return disp_vis
    
    def display_loop(self):
        # Cria uma janela que será usada para exibir a imagem
        cv2.namedWindow("Mapa de Disparidade V0")

        while rclpy.ok():
            # Pega a imagem mais recente para exibir.
            # Usamos o 'with self.lock:' para garantir que o callback do ROS
            # não tente escrever na variável ao mesmo tempo que estamos lendo.
            with self.lock:
                display_image = self.latest_disparity_map
            
            # Se já recebemos algum mapa de disparidade...
            if display_image is not None:
                # Lembre-se de normalizar se não for um uint8!
                # if display_image.dtype != np.uint8:
                #    display_image = cv2.normalize(...)
                
                cv2.imshow("Mapa de Disparidade V0", display_image)
                self.get_logger().info("Janela atualizada")
                cv2.imshow("Img Left", self.imgL_cv2)
            # A LINHA MAIS IMPORTANTE!
            # Processa os eventos da GUI por 1ms. Sem isso, a janela congela.
            key = cv2.waitKey(1) & 0xFF
            
            # Permite fechar a janela com a tecla 'q'
            if key == ord('q'):
                break

            # Uma pequena pausa para não sobrecarregar a CPU com este loop
            time.sleep(0.01)

        # Limpeza ao sair do loop
        cv2.destroyAllWindows()

    def draw_yolo_boxes(self, image, detections):
        yolo = detections.yolov8_inference
        img_out = image.copy()
        color=(60, 20, 220)
        
        for det in yolo:
            x1 = det.top
            y1 = det.left
            x2 = det.bottom
            y2 = det.right
            
            # Desenhar o retângulo
            cv2.rectangle(img_out, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
        return img_out

def main(args=None):
    rclpy.init(args=args)
    
    disp_map_show = NoDisparidadeShowWindow()
    
    rclpy.spin(disp_map_show)
    disp_map_show.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    cv2.destroyAllWindows()
    