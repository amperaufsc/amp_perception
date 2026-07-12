import numpy as np
from fs_msgs.msg import TrackStampedWithCovariance, Track, ConeWithCovariance
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from sensor_msgs.msg import Image
import os
import yaml
import cv2
from cv_bridge import CvBridge
from pathlib import Path

bridge = CvBridge()

class PerceptionProcess:
    # perception_calc(endereço_arq_yaml, disp_img).triangulacao(baseline,yoloinference) = ((X,Y,Z)) -> Posicao do cone no espaco 3D.
    def __init__(self, baseline, left_config_file_name, right_config_file_name):

        left_config_path = os.path.join(
            get_package_share_directory('perception'),
            'config', 
            left_config_file_name
        )
        right_config_path = os.path.join(
            get_package_share_directory('perception'),
            'config', 
            right_config_file_name
        )

        self.left_config_yaml = PerceptionProcess.yaml_reader(left_config_path)
        self.right_config_yaml = PerceptionProcess.yaml_reader(right_config_path)

        self.focal_length_x = self.left_config_yaml[0][0][0]
        self.focal_length_y = self.left_config_yaml[0][1][1]
        self.center_x = self.left_config_yaml[0][0][2]
        self.center_y = self.left_config_yaml[0][1][2]

        self.baseline = baseline

    def object_on_map(self, yoloinference, disp_map, imgL_raw_ros_msg, imgR_raw_ros_msg):  
        
        height = imgL_raw_ros_msg.height
        width = imgL_raw_ros_msg.width

        imgL_raw_ros_msg = bridge.imgmsg_to_cv2(imgL_raw_ros_msg)
        imgR_raw_ros_msg = bridge.imgmsg_to_cv2(imgR_raw_ros_msg)
        
        is_disp_map = None

        cone_list = []
        bb_yolo = yoloinference.yolov8_inference
        
        for box in bb_yolo:
            cone = ConeWithCovariance()
            cor = box.class_name
            confidence = box.confidence
            
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
            
            sample_w = max(1, (bb_w * 0.15)//2)
            sample_h = max(1, (bb_h * 0.2)//2)
            
            obj_x1 = int(max(0, center_x - sample_w))
            obj_y1 = int(max(0, center_y - sample_h))
            obj_x2 = int(min(width, center_x + sample_w))
            obj_y2 = int(min(height, center_y + sample_h))
            
            roi = disp_map[obj_y1:obj_y2, obj_x1:obj_x2]
            valid = roi[np.isfinite(roi)]
            valid = valid[valid > 0]

            if len(valid) == 0:
                continue

            median_disp = np.median(valid)

            if median_disp > 700:
                Z = median_disp / 1000
                X, Y = self.x_y_space_measure(Z, center_x, center_y)
                is_disp_map = True

            else:
                X,Y,Z = self.triangulacao(center_y, center_x, median_disp, disp_map, imgL_raw_ros_msg, imgR_raw_ros_msg)
                is_disp_map = False

            if not (np.isfinite(X) and np.isfinite(Y) and np.isfinite(Z)):
                continue

            if Z >= 5 or Z == 0:
                continue

            cone.location.x = float(X)
            cone.location.y = float(Y)
            cone.location.z = float(Z)

            deviationZ = 0.0096*cone.location.z + 0.1643   #linearização do erro da detecção vs distancia no eixo z
            deviationX = 0.0232*cone.location.x + 0.1204   #linearização do erro da detecção vs distancia no eixo x
            deviation = np.sqrt(deviationX**2 + deviationZ**2)   
            
            cone.deviation = deviation
            cone.confidence = confidence

            cone_list.append(cone)
              
        cone_track = TrackStampedWithCovariance()
        cone_track.track = cone_list

        return (cone_track, is_disp_map)


    def x_y_space_measure(self, Z_point, center_x, center_y):
        X_point = (center_x-self.center_x) * Z_point / self.focal_length_x
        Y_point = (center_y-self.center_y) * Z_point / self.focal_length_y  
        return X_point, Y_point
        
    def triangulacao(self, center_y, center_x, disparity, disp_map, imgL_raw_ros_msg, imgR_raw_ros_msg):
        
        Z = (self.baseline * self.focal_length_x ) / disparity
        Z = Z 
        X, Y = self.x_y_space_measure(Z, center_x, center_y)
        
        if np.isfinite(X) and np.isfinite(Y) and np.isfinite(Z):
            return float(X), float(Y), float(Z)
        else:
            return 0.0, 0.0, 0.0

    def triangulacao_lux(self, center_y, center_x, disparity, disp_map, imgL_raw_ros_msg, imgR_raw_ros_msg):
        
        Z = (self.baseline * self.focal_length_x ) / disparity
        Z = Z *16
        X, Y = self.x_y_space_measure(Z, center_x, center_y)
        
        if np.isfinite(X) and np.isfinite(Y) and np.isfinite(Z):
            return float(X), float(Y), float(Z)
        else:
            return 0.0, 0.0, 0.0
    
    def approximate_stereo_rectify(self, imgL, imgR):

        imgL_cv = bridge.imgmsg_to_cv2(imgL)
        imgR_cv = bridge.imgmsg_to_cv2(imgR)

        image_size = (imgL_cv.shape[1], imgL_cv.shape[0])

        [k_left, d_left, r_left, p_left] = self.left_config_yaml
        [k_right, d_right, r_right, p_right] = self.right_config_yaml

        mapLx, mapLy = cv2.initUndistortRectifyMap(k_left, d_left, r_left, p_left, image_size, cv2.CV_32FC1)
        mapRx, mapRy = cv2.initUndistortRectifyMap(k_right, d_right, r_right, p_right, image_size, cv2.CV_32FC1)

        rectL_cv = cv2.remap(imgL_cv, mapLx, mapLy, cv2.INTER_LINEAR)
        rectR_cv = cv2.remap(imgR_cv, mapRx, mapRy, cv2.INTER_LINEAR)

        rectL_raw = bridge.cv2_to_imgmsg(rectL_cv, encoding=imgL.encoding)
        rectR_raw = bridge.cv2_to_imgmsg(rectR_cv, encoding=imgR.encoding)

        return rectL_raw, rectR_raw
    
    def DisparityProcess(self, imgL_ros_msg, imgR_ros_msg):
        
        imgL_cv = bridge.imgmsg_to_cv2(imgL_ros_msg)
        imgR_cv = bridge.imgmsg_to_cv2(imgR_ros_msg)

        imgL_cv = cv2.cvtColor(imgL_cv, cv2.COLOR_BGR2GRAY)
        imgR_cv = cv2.cvtColor(imgR_cv, cv2.COLOR_BGR2GRAY)

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
        stereo = stereo.compute(imgL_cv, imgR_cv).astype(np.float32) / 16

        disp_map = cv2.normalize(stereo,None, 0, 255, cv2.NORM_MINMAX)
        disp_map = np.uint8(disp_map)

        return (disp_map, stereo)
    
    @staticmethod
    def yaml_reader(path):
        try:
            with open(path, 'r') as file:
                
                data = yaml.safe_load(file)
                kL = np.array(data['camera_matrix']['data'], dtype=np.float64)
                dL = np.array(data['distortion_coefficients']['data'], dtype=np.float64)
                rL = np.array(data['rectification_matrix']['data'], dtype=np.float64)
                pL = np.array(data['projection_matrix']['data'], dtype=np.float64)
                
                return [kL, dL, rL, pL]
                
        except FileNotFoundError:
            print("ERRO: Arquivo YAML não encontrado no endereco")
            print(path)
            return None
        except KeyError:
            print("ERRO: Palavra-chave não encontrada no arquivo")
            return None
        except Exception as e:
            print("ERRO inesperado ao ler YAML: {e}")
            return None