import numpy as np
from fs_msgs.msg import TrackStamped, Track, Cone
import os
import yaml
import cv2
from cv_bridge import CvBridge
bridge = CvBridge()

class PerceptionProcess:
    # perception_calc(endereço_arq_yaml, disp_img).triangulacao(baseline,yoloinference) = ((X,Y,Z)) -> Posicao do cone no espaco 3D.
    def __init__(self, endereco_matriz_intrinsica_left, endereco_matriz_intrinsica_right, baseline):
        self.camera_matrix = self.yaml_reader(endereco_matriz_intrinsica_left, endereco_matriz_intrinsica_right)
        self.baseline = baseline
        
    def object_on_map(self, yoloinference, disp_map, imgL_raw_ros_msg, imgR_raw_ros_msg):
        
        imgL_raw_ros_msg = imgL_raw_ros_msg
        imgR_raw_ros_msg = imgR_raw_ros_msg
        
        cone_list = []
        disp_map = disp_map
        bb_yolo = yoloinference.yolov8_inference
        
        for box in bb_yolo:
            cone = Cone()
            cor = box.class_name
            x1 = box.top
            y1 = box.left
            x2 = box.bottom
            y2 = box.right
            
            pixel_y = int((abs(y2-y1) / 2) + min(y1, y2))
            pixel_x = int((abs(x2-x1) / 2) + min(x1, x2))
            
            roi = disp_map[(pixel_y - 15//2):(pixel_y + 15//2),(pixel_x - 15//2):(pixel_x + 15//2)]
            valid = roi[np.isfinite(roi)]
            valid = valid[valid > 0]
            median_disp = np.median(valid)
            
            if median_disp >= 85:
                Y,X,Z = self.triangulacao(pixel_y, pixel_x, median_disp, disp_map, imgL_raw_ros_msg, imgR_raw_ros_msg)
                cone.location.x = X
                cone.location.y = Y
                cone.location.z = Z
                if cor == 'blue_cone':
                    cone.color=0
                elif cor == 'yellow_cone':
                    cone.color=1
                elif cor == 'large_orange_cone':
                    cone.color=2
                if cone.location.z < 20:
                    cone_list.append(cone)
        cone_track = Track()
        cone_track.track = cone_list

        return (cone_track, median_disp)
            
            
        
    def triangulacao(self, pixel_y, pixel_x, disparity, disp_map, imgL_raw_ros_msg, imgR_raw_ros_msg):
        
        imgL_cv = bridge.imgmsg_to_cv2(imgL_raw_ros_msg,desired_encoding='bgr8')
        imgR_cv = bridge.imgmsg_to_cv2(imgR_raw_ros_msg,desired_encoding='bgr8')
         
        focal_length_x = self.camera_matrix[0][0][0][0]
        focal_length_y = self.camera_matrix[0][0][1][1]
        
        center_x = self.camera_matrix[0][0][0][2]
        center_y = self.camera_matrix[0][0][1][2]
        
        Z = (self.baseline * focal_length_x ) / disparity
        Z = Z*6
        X = (pixel_x-(center_x/2)) * Z / focal_length_x
        Y = (pixel_y-center_y) * Z / focal_length_y   

        #none1, none2, Q = self.approximate_stereo_rectify(imgL_cv, imgR_cv)
        #points_3D = cv2.reprojectImageTo3D(disp_map, Q)
        
        #X = points_3D[pixel_y, pixel_x, 0]
        #Y = points_3D[pixel_y, pixel_x, 1]
        #Z = points_3D[pixel_y, pixel_x, 2]
        
        if np.isfinite(X) and np.isfinite(Y) and np.isfinite(Z):
            return float(X), float(Y), float(Z)
        else:
            return 0.0, 0.0, 0.0
        
    def approximate_stereo_rectify(self, imgL, imgR):
        
        (kL, dL, rL, pL) = self.camera_matrix[0]
        (kR, dR, rR, pR) = self.camera_matrix[1]
        
        kL = np.array(kL).reshape(3,3)
        dL = np.array(dL)

        # --- RIGHT ---
        kR = np.array(kR).reshape(3,3)
        dR = np.array(dR)
        
        R = np.eye(3, dtype=np.float64)

        T = np.array([-self.baseline, 0, 0], dtype=np.float64)

        image_size = (imgL.shape[1], imgL.shape[0])
        
        rL, rR, pL, pR, Q, roi1, roi2 = cv2.stereoRectify(
            kL, dL,
            kR, dR,
            image_size,
            R, T,
            flags=cv2.CALIB_ZERO_DISPARITY,
            alpha=0
        )

        # MAPAS DE REMAP
        mapLx, mapLy = cv2.initUndistortRectifyMap(
            kL, dL, rL, pL, image_size, cv2.CV_32FC1
        )
        mapRx, mapRy = cv2.initUndistortRectifyMap(
            kR, dR, rR, pR, image_size, cv2.CV_32FC1
        )

        # APLICA REMAP
        rectL = cv2.remap(imgL, mapLx, mapLy, cv2.INTER_LINEAR)
        rectR = cv2.remap(imgR, mapRx, mapRy, cv2.INTER_LINEAR)

        return (rectL, rectR, Q)

    def DisparityProcess(self, imgL_ros_msg, imgR_ros_msg):
        
        imgL_cv = bridge.imgmsg_to_cv2(imgL_ros_msg, desired_encoding='bgr8')
        imgR_cv = bridge.imgmsg_to_cv2(imgR_ros_msg, desired_encoding='bgr8')

        #deixa a imagem em preto e branco.
        imgL_cv = cv2.cvtColor(imgL_cv,cv2.COLOR_BGR2GRAY)
        imgR_cv = cv2.cvtColor(imgR_cv,cv2.COLOR_BGR2GRAY)

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
        disp_map = stereo.compute(imgL_cv, imgR_cv).astype(np.float32)

        # Calculating disparith using the StereoSGBM algorithm

        disp_map = cv2.normalize(disp_map,None, 0, 255, cv2.NORM_MINMAX)
        disp_map = np.uint8(disp_map)
        disp_map[disp_map < 0] = 0
        disp_map[disp_map > 64] = 64
        disp_vis = (disp_map / np.max(disp_map) * 255).astype(np.uint8)
        disp_vis = cv2.medianBlur(disp_vis, 5)  

        return disp_vis



    
    def yaml_reader(self, endereco_left, endereco_right):
        try:
            saida = [None,None]
            with open(endereco_left, 'r') as f:
                data = yaml.safe_load(f)

                kL = np.array(data['camera_matrix']['data'], dtype=np.float64).reshape(3,3)
                dL = np.array(data['distortion_coefficients']['data'], dtype=np.float64)
                rL = np.array(data['rectification_matrix']['data'], dtype=np.float64).reshape(3,3)
                pL = np.array(data['projection_matrix']['data'], dtype=np.float64).reshape(3,4)

                saida[0] = (kL, dL, rL, pL)
                
            with open(endereco_right, 'r') as f:
                data = yaml.safe_load(f)

                kR = np.array(data['camera_matrix']['data'], dtype=np.float64).reshape(3,3)
                dR = np.array(data['distortion_coefficients']['data'], dtype=np.float64)
                rR = np.array(data['rectification_matrix']['data'], dtype=np.float64).reshape(3,3)
                pR = np.array(data['projection_matrix']['data'], dtype=np.float64).reshape(3,4)

                saida[1] = (kR, dR, rR, pR)

            return saida
            
        except FileNotFoundError:
            print("ERRO: Arquivo YAML não encontrado no endereco")
            return None
        except KeyError:
            print("ERRO: Palavra-chave não encontrada no arquivo")
            return None
        except Exception as e:
            print("ERRO inesperado ao ler YAML: {e}")
            return None