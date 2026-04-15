import numpy as np
from fs_msgs.msg import TrackStampedWithCovariance, Track, ConeWithCovariance
from sensor_msgs.msg import Image
import os
import yaml
import cv2
from cv_bridge import CvBridge
bridge = CvBridge()

class PerceptionProcess:
    # perception_calc(endereço_arq_yaml, disp_img).triangulacao(baseline,yoloinference) = ((X,Y,Z)) -> Posicao do cone no espaco 3D.
    def __init__(self, baseline):

        endereco_matriz_intrinsica_left = '/home/otaviogoulart/ws/src/amp_perception/perception/config/OAKDLR_left.yaml'
        endereco_matriz_intrinsica_right = '/home/otaviogoulart/ws/src/amp_perception/perception/config/OAKDLR_right.yaml'
        self.camera_matrix = self.yaml_reader(endereco_matriz_intrinsica_left, endereco_matriz_intrinsica_right)

            #OAK D LR
        self.focal_length_x = self.camera_matrix[0][0][0][0]
        self.focal_length_y = self.camera_matrix[0][0][1][1]
        self.center_x = self.camera_matrix[0][0][0][2]
        self.center_y = self.camera_matrix[0][0][1][2]

            #OAK D W
        #self.focal_length_x = 574.92
        #self.focal_length_y = 574.92
        #self.center_x = 639.78
        #self.center_y = 377.21

        self.baseline = baseline

    def object_on_map(self, yoloinference, disp_map, imgL_raw_ros_msg, imgR_raw_ros_msg):  
        
        height = imgL_raw_ros_msg.height
        width = imgL_raw_ros_msg.width

        imgL_raw_ros_msg = bridge.imgmsg_to_cv2(imgL_raw_ros_msg)
        imgR_raw_ros_msg = bridge.imgmsg_to_cv2(imgR_raw_ros_msg)
        
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
            
            median_disp = np.median(valid)
            
            if len(valid) >= 1:
                if median_disp > 300:
                    Z = median_disp / 1000
                    X, Y = self.x_y_space_measure(Z, center_x, center_y)
                    is_disp_map = True

                else:
                    X,Y,Z = self.triangulacao(center_y, center_x, median_disp, disp_map, imgL_raw_ros_msg, imgR_raw_ros_msg)
                    is_disp_map = False

            cone.location.x = X
            cone.location.y = Y
            cone.location.z = Z
            
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

        [kL, dL, rL_yaml, pL_yaml] = self.camera_matrix[0]
        [kR, dR, rR_yaml, pR_yaml] = self.camera_matrix[1]

        R = np.array([
            1.0000, -0.0014,  0.0045,
            0.0014,  1.0000, -0.0008,
            -0.0045,  0.0008,  1.0000
        ]).reshape((3, 3))

        T = np.array([-147.9689, 0.4888, 0.4957])

        R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
            cameraMatrix1=kL,
            distCoeffs1=dL,
            cameraMatrix2=kR,
            distCoeffs2=dR,
            imageSize=image_size,
            R=R,
            T=T,
            flags=cv2.CALIB_ZERO_DISPARITY,
            alpha=1
        )

        mapLx, mapLy = cv2.initUndistortRectifyMap(kL, dL, R1, P1, image_size, cv2.CV_32FC1)
        mapRx, mapRy = cv2.initUndistortRectifyMap(kR, dR, R2, P2, image_size, cv2.CV_32FC1)

        rectL_cv = cv2.remap(imgL_cv, mapLx, mapLy, cv2.INTER_LINEAR)
        rectR_cv = cv2.remap(imgR_cv, mapRx, mapRy, cv2.INTER_LINEAR)

        rectL_raw = bridge.cv2_to_imgmsg(rectL_cv, encoding=imgL.encoding)
        rectR_raw = bridge.cv2_to_imgmsg(rectR_cv, encoding=imgR.encoding)

        return rectL_raw, rectR_raw
    
    def DisparityProcess(self, imgL_ros_msg, imgR_ros_msg):
        
        imgL_cv = bridge.imgmsg_to_cv2(imgL_ros_msg)
        imgR_cv = bridge.imgmsg_to_cv2(imgR_ros_msg)

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
    
    def yaml_reader(self, endereco_left, endereco_right):
        try:
            saida = [None, None]
            with open(endereco_left, 'r') as f:
                
                data = yaml.safe_load(f)
                kL = np.array(data['camera_matrix']['data']).reshape((3, 3))
                dL = np.array(data['distortion_coefficients']['data']).reshape((1, -1))
                rL = np.array(data['rectification_matrix']['data']).reshape((3, 3))
                pL = np.array(data['projection_matrix']['data']).reshape((3, 4))

                saida[0] = (kL, dL, rL, pL)
            
            with open(endereco_right, 'r') as f:

                data = yaml.safe_load(f)
                kR = np.array(data['camera_matrix']['data']).reshape((3, 3))
                dR = np.array(data['distortion_coefficients']['data']).reshape((1, -1))
                rR = np.array(data['rectification_matrix']['data']).reshape((3, 3))
                pR = np.array(data['projection_matrix']['data']).reshape((3, 4))

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