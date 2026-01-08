import cv2
import torch.nn as nn
import numpy as np
import torch
from PIL import Image
from ultralytics import YOLO
from torchvision import transforms

class KeypointsEstimator:
    def __init__(self,camera_info,keypoints_path,distortion):
        self.camera_distortion= distortion
        self.model_keypoints = KeyPoints_Net()
        self.keypoints_net = self.model_keypoints.load_state_dict(torch.load(keypoints_path, map_location=torch.device('cuda')))
        self.model_keypoints.eval()
        self.model_keypoints.cuda()
        self.object3D_points = np.array([[0.0,0.0 , 227.0], [0.0, -37.0, 144.0], [0.0, 37.0, 144.0], [0.0, -48.0, 78.0], [0.0, 48.0, 78.0], [0.0, -60.0, 0.0], [0.0, 60.0, 0.0]], dtype=np.float32)
        self.camera_intrinsics=camera_info #garantir que seja de fato os intrisicos da camera
    
    
    def get_image_keypoints(self,boundingbox_list,image):
        keypoints_list=[]
        
        
        
        for box in boundingbox_list:
            x1,y1 = int(box.top),int(box.left)
            x2,y2 = int(box.bottom),int(box.right)
            
            crop = image[y1:y2,x1:x2]
            imagem_redimencionada=cv2.resize(crop,(80,80))
            im = Image.fromarray(cv2.cvtColor(imagem_redimencionada, cv2.COLOR_BGR2RGB))
            test_transforms = transforms.Compose([transforms.Resize((80, 80)), transforms.ToTensor()])
            im = test_transforms(im)
            pytorch_image = im.unsqueeze(0).to('cuda')
            result = self.model_keypoints(pytorch_image)
            result = result.tolist()[0]
            keypoints_x,keypoints_y = result[0],result[1]
            largura_orig=x2-x1
            altura_orig= y2-y1
            xy_imag=[]
            for j, (x, y) in enumerate(zip(result[::2], result[1::2])):
                x_keypoint = int(((x / 80) * (x2 - x1)) + x1)
                y_keypoint = int(((y / 80) * (y2 - y1)) + y1)

                xy_imag.append([x_keypoint,y_keypoint])
           
            keypoints_list.append(xy_imag)
        return keypoints_list

    

    def get_position_estimation(self, image,yolo_detections):
        cameraMatrix = self.camera_intrinsics
        
        #bounding_boxes = self.yolo_model.predict(image)

        boundingbox_list = yolo_detections
        keypoints = self.get_image_keypoints(boundingbox_list,image)
        cameraMatrix = np.array(cameraMatrix, dtype=np.float32)

        obstacles = list()
        rvec_list=[]
        tvec_list=[]
        
        
        for i,objects in enumerate(keypoints):
            
            xy_imag = np.array([objects], dtype=np.float32)
            funciona,rvec,tvec= cv2.solvePnP(self.object3D_points, xy_imag, cameraMatrix, np.array([]),flags=0)
            obstacles.append((tvec[0][0],tvec[2][0])) 
            rvec_list.append(rvec)
            tvec_list.append(tvec)
            
            
            
        
        return obstacles,rvec_list,tvec_list










class ResNet(nn.Module):
    def __init__(self, in_channels, out_channels):
        super(ResNet, self).__init__()

        self.conv1 = nn.Conv2d(in_channels=in_channels, out_channels=out_channels, kernel_size=3, stride=2, padding=4)
        self.bn1 = nn.BatchNorm2d(out_channels)
        self.relu1 = nn.ReLU()
        self.conv2 = nn.Conv2d(in_channels=out_channels, out_channels=out_channels, kernel_size=3, stride=2, padding=4)
        self.bn2 = nn.BatchNorm2d(out_channels)
        self.relu2 = nn.ReLU()

        self.shortcut_conv = nn.Conv2d(in_channels=in_channels, out_channels=out_channels, kernel_size=1, stride=1)
        self.shortcut_bn = nn.BatchNorm2d(out_channels)

    def forward(self, x):
        c1 = self.conv1(x)
        b1 = self.bn1(c1)
        act1 = self.relu1(b1)
        out = self.bn2(self.conv2(act1))
        return out

class KeyPoints_Net(nn.Sequential):
    def __init__(self):
        super().__init__(
           nn.Conv2d(in_channels=3,out_channels=64,kernel_size=(11,11), stride=10, dilation=1),
            nn.BatchNorm2d(64),
            nn.ReLU(),
            ResNet(in_channels=64, out_channels=64),
            ResNet(in_channels=64, out_channels=128),
            ResNet(in_channels=128, out_channels=256),
            ResNet(in_channels=256, out_channels=512),
            nn.Flatten(),
            nn.Linear(in_features=(512*7*7), out_features = 14))