import cv2 
from fs_msgs.msg import TrackStampedWithCovariance, Track
from fs_msgs.msg import ConeWithCovariance, Cone
import numpy as np
import rclpy

class DisparityEstimator:
    def __init__(self,intrinsics, set_disparity):
        self.intrinsic_matrix=intrinsics['camera_matrix']['data']
        self.set_disparity = set_disparity

    def get_object_on_map(self,left_img,disp_map,yolo,baseline,focal_length):
        cone_list=[]

        self.bb_on_left=yolo
        self.left_img=left_img
        self.disp_map=disp_map
        
        track=TrackStampedWithCovariance()
        
        for box in self.bb_on_left:
            cone = ConeWithCovariance()
            cor=box.class_name
            centro_x=int((int(box.top)+int(box.bottom))/2)
            centro_y=int((int(box.left)+int(box.right))/2)
            lado=15
            disp_map=np.array(disp_map)
        
            disp_slice=disp_map[(centro_y-lado//2):(centro_y+lado//2),(centro_x-lado//2):(centro_x+lado//2)]
        
            disparity=np.median(disp_slice)

            confidence = box.confidence
            
            if disparity >= 85:
                X,Y,Z=self.triangulation(self.disp_map,centro_x,centro_y,baseline,focal_length)
                cone.location.x = X
                cone.location.y = Y
                cone.location.z = Z
                if cor == 'blue_cone':
                    cone.color=0
                elif cor == 'yellow_cone':
                    cone.color=1
                elif cor == 'large_orange_cone':
                    cone.color=2
                # if cone.location.z < 20:
                #     cone_list.append(cone)

                deviationZ = 0.0096*cone.location.z + 0.1643   #linearização do erro da detecção vs distancia no eixo z
                deviationX = 0.0232*cone.location.x + 0.1204   #linearização do erro da detecção vs distancia no eixo x
                deviation = np.sqrt(deviationX**2 + deviationZ**2)
                cone.deviation = deviation
                cone.confidence = confidence
                
                if cone.location.z < 5:
                    cone_list.append(cone)                
        
            # cv2.circle(left_img,(centro_x,centro_y),1,(0,0,255))
            # cv2.putText(left_img,str(Z),(centro_x,centro_y),cv2.FONT_HERSHEY_COMPLEX,1,(255,255,255),2,cv2.LINE_AA)
            # cv2.imshow('left image',left_img)
            # cv2.waitKey(0)
        track.track=cone_list
        
        return track
    
    def triangulation(self,disp_map,ponto_x,ponto_y,baseline,focal_length):
        lado=15
        disp_map=np.array(disp_map)
        
        disp_slice=disp_map[(ponto_y-lado//2):(ponto_y+lado//2),(ponto_x-lado//2):(ponto_x+lado//2)]
        
        disparity=np.median(disp_slice)

        cx=self.intrinsic_matrix[0][2]
        cy=self.intrinsic_matrix[1][2]
        
        focal_length_x=self.intrinsic_matrix[0][0]
        focal_length_y=self.intrinsic_matrix[1][1]
        
        baseline=0.150
    
        #rclpy.get_logger().info(disparity)
        
        if self.set_disparity:
            
            Z = disparity
            Z = Z/1000   
        else:
            Z=((baseline*focal_length_x)/disparity)
            Z= Z*16 

        X=(((ponto_x-cx)*Z)/focal_length_x)
        Y=(((ponto_y-cy)*Z)/focal_length_y)
        
        return X,Y,Z
    
    def track_stamped_with_covariance_msg_compose(self, cone_track, header):

        track_stamped = TrackStampedWithCovariance()
        track_stamped.header = header
        track_stamped.track = cone_track.track
        return track_stamped


        
        