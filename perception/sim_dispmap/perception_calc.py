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
        camera_matrix = self.yaml_reader(endereco_matriz_intrinsica_left, endereco_matriz_intrinsica_right)
        self.baseline = baseline
        
        self.focal_length_x = camera_matrix[0][0][0][0]
        self.focal_length_y = camera_matrix[0][0][1][1]
        self.center_x = camera_matrix[0][0][0][2]
        self.center_y = camera_matrix[0][0][1][2]
        
    def object_on_map(self, yoloinference, disp_map, imgL_raw_ros_msg, imgR_raw_ros_msg):
        
        imgL_raw_ros_msg = imgL_raw_ros_msg
        imgR_raw_ros_msg = imgR_raw_ros_msg
        
        cone_list = []
        disp_map = disp_map
        bb_yolo = yoloinference.yolov8_inference
        
        for box in bb_yolo:
            cone = Cone()
            cor = box.class_name
            
            if cor == 'blue_cone':
                cone.color=0
            elif cor == 'yellow_cone':
                cone.color=1
            elif cor == 'large_orange_cone':
                cone.color=2
            
            x1 = box.top
            y1 = box.left
            x2 = box.bottom
            y2 = box.right
            
            center_y = int((abs(y2-y1) / 2) + min(y1, y2))
            center_x = int((abs(x2-x1) / 2) + min(x1, x2))
            
            bb_w = x2-x1
            bb_h = y2-y1
            
            sample_w = max(1, (bb_w * 0.2)//2)
            sample_h = max(1, (bb_h * 0.2)//2)
            
            obj_x1 = int(max(0, center_x - sample_w))
            obj_y1 = int(max(0, center_y - sample_h))
            obj_x2 = int(min(disp_map.shape[1], center_x + sample_w))
            obj_y2 = int(min(disp_map.shape[0], center_y + sample_h))
            
            roi = disp_map[obj_y1:obj_y2, obj_x1:obj_x2]
            valid = roi[np.isfinite(roi)]
            valid = valid[valid > 0]
            
            median_disp = 0.0
            median_disp = np.median(valid)
            
            if len(valid) >= 1:
                X,Y,Z = self.triangulacao(center_y, center_x, median_disp, disp_map, imgL_raw_ros_msg, imgR_raw_ros_msg)
                cone.location.x = X
                cone.location.y = Y
                cone.location.z = Z
                if cone.location.z < 20:
                    cone_list.append(cone)      
        cone_track = Track()
        cone_track.track = cone_list

        return (cone_track, median_disp)
            
    def x_y_space_measure(self, Z_point, center_x, center_y):
        X_point = (center_x-(self.center_x/2)) * Z_point / self.focal_length_x
        Y_point = (center_y-self.center_y) * Z_point / self.focal_length_y  
        return X_point, Y_point
        
    def triangulacao(self, center_y, center_x, disparity, disp_map, imgL_raw_ros_msg, imgR_raw_ros_msg):
        
        imgL_cv = bridge.imgmsg_to_cv2(imgL_raw_ros_msg,desired_encoding='bgr8')
        imgR_cv = bridge.imgmsg_to_cv2(imgR_raw_ros_msg,desired_encoding='bgr8')
        
        Z = (self.baseline * self.focal_length_x ) / disparity
        Z = Z*6 
        X, Y = self.x_y_space_measure(Z, center_x, center_y)
        
        if np.isfinite(X) and np.isfinite(Y) and np.isfinite(Z):
            return float(X), float(Y), float(Z)
        else:
            return 0.0, 0.0, 0.0
    
    def monocular_measure(self, center_x, center_y, cone_height, bb_h):
        Z = (self.focal_length_x  * cone_height) / bb_h
        X, Y = self.x_y_space_measure(Z, center_x, center_y)
        return X, Y, Z
    
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