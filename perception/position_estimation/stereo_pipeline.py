import cv2
from keypoints_estimator import KeypointsEstimator
import numpy as np

class StereoPipeline:
    def __init__(self,cam_left_info,cam_right_info):
        self.left_intrinsics=cam_left_info['rectification_matrix']['data']
        self.camera_distortion=cam_left_info['distortion_coefficients']['data']
        self.right_intrinsics=cam_right_info['camera_matrix']['data']
        self.sift = cv2.SIFT.create()
        self.bf = cv2.BFMatcher()


    def get_object_position(self, img_left,img_right,yolo,keypoints_path):
        keypoints = KeypointsEstimator(self.left_intrinsics,keypoints_path,self.camera_distortion)
        objects,rvec_list,tvec_list=keypoints.get_position_estimation(img_left,yolo)
        boundingboxes=yolo
        objec3D=keypoints.object3D_points
        ponto1,ponto2,ponto3,ponto4=self.transfer_detection(tvec_list,rvec_list,boundingboxes,objec3D,img_left,img_right)
        box_right=[ponto1,ponto2]
        box_left=[ponto3,ponto4]

        good_points_left,good_points_right=self.SIFT_and_Matcher(box_left,box_right,img_left,img_right)
        objects= self.triangulation(good_points_left,good_points_right)
        return objects
    
    
    def transfer_detection(self,tvec_list,rvec_list,boundingboxes,object3D,img_left,img_right):
        for i,tvec in enumerate(tvec_list):
            soma_x=0
            soma_y=0
            rvec=rvec_list[i]
            image_points, _ = cv2.projectPoints(object3D, rvec, tvec, np.array(self.left_intrinsics), np.array(self.camera_distortion))
            for point in image_points.astype(int):
        
                soma_x=soma_x+point[0][0]
                soma_y=soma_y+point[0][1]

            centrobb=(int(soma_x/7),int(soma_y/7))
    
            xyxy=(boundingboxes[i])
            
            
            altura=int(xyxy.top-xyxy.bottom)
            largura=int(xyxy.left-xyxy.right)
            ponto1 = (int(centrobb[0] + altura/2), int(centrobb[1] + largura/2))
            ponto2 = (int(centrobb[0] - altura/2), int(centrobb[1] - largura/2))
            ponto3 = (int(xyxy.top),int(xyxy.left))
            ponto4 = (int(xyxy.bottom),int(xyxy.right))
            cv2.rectangle(img_left,(ponto3),(ponto4),(0,255,0),3)
            cv2.imshow('img left',img_left)
            cv2.rectangle(img_right,(ponto1),(ponto2),(0,255,0),3)
            cv2.imshow('img right',img_right)
        
            cv2.waitKey(0)
                    

        return ponto1,ponto2,ponto3,ponto4
    
    def SIFT_and_Matcher(self,pontos_right,pontos_left,img_left,img_right):
        raw_img_left=img_left
        raw_img_right=img_right
        ponto1,ponto2=pontos_left
        ponto3,ponto4=pontos_right
        
        x1l,y1l = ponto1
        
        x2l,y2l = ponto2
        
        x1r,y1r = ponto3
        
        x2r,y2r = ponto4
        
        
        

        img_left = img_left[y1l:y2l,x1l:x2l]
        img_right= img_right[y1r:y2r,x1r:x2r]
        
        kp1, des1 = self.sift.detectAndCompute(img_left,None)
        kp2, des2 = self.sift.detectAndCompute(img_right,None)
    
        
        
        matches = self.bf.knnMatch(des1,des2,k=2)
        print(matches)
        

        good = []
        for m, n in matches:
            if m.distance < 0.7 * n.distance:  
                good.append([m])
        if good:
            good=good[0]

        if good:
            points1 = np.array([kp1[good[0].queryIdx].pt])
            points2 = np.array([kp2[good[0].trainIdx].pt])

            points1 = points1.reshape(-1, 1, 2)
            points2 = points2.reshape(-1, 1, 2)
            return points1,points2
        else:
            return [],[]
        
    def triangulation(self,points1,points2):
        K = np.array(self.left_intrinsics)
        K2 = np.array(self.right_intrinsics)
        R = np.eye(3)  
        T = np.array([0, 0, 0])  
        RT = np.hstack((R, T.reshape(-1, 1)))
        P1 = np.dot(K, RT)
      
        T2 = np.array([7.5, 0, 0])  
        RT2 = np.hstack((R, T2.reshape(-1, 1)))
        P2 = np.dot(K2, RT2)
        print(points1)
        print(points2)
        
        obj = cv2.triangulatePoints(P1, P2, points1, points2)   
        obj = [obj[0]/obj[3],obj[1]/obj[3],obj[2]/obj[3]]
        return obj
        
            

        
