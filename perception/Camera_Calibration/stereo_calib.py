import cv2
import numpy as np
import yaml

def reader(file_dir):
    with open(file_dir, "r") as file:
        data = yaml.safe_load(file)

        K = np.array(data['camera_matrix']['data'], dtype=np.float64)
        D = np.array(data['distortion_coefficients']['data'], dtype=np.float64)

        try: 
            R = np.array(data['rotation_matrix']['data'], dtype=np.float64)
            T = np.array(data['translation_matrix']['data'], dtype=np.float64)

            return K, D, R, T
        except:
            return K, D
    
k1,d1, R, T = reader('camera_info_left.yaml')
k2,d2 = reader('camera_info_right.yaml')

imageSize = [768, 480]

[R1, R2, P1, P2, Q, roi1, roi2] = cv2.stereoRectify(
    cameraMatrix1=k1, 
    distCoeffs1=d1, 
    cameraMatrix2=k2, 
    distCoeffs2=d2, 
    imageSize=imageSize, 
    R=R, 
    T=T, 
    alpha=1 # 0 para cortar bordas pretas, 1 para manter tudo
)

data = [R1, R2, P1, P2, Q, roi1, roi2]

for i in range(0, 4):
    print (data[i])

