#!/usr/bin/env python3
import depthai as dai 
import cv2 
import numpy as np
from ultralytics import YOLO


class ConeDetection():
    def __init__(self,yolopath):
        
        self.model = YOLO(yolo_path)
        
   
    
    def get_boundingbox(self,image):
        bounding_boxes= self.model.predict(image)
        conf_list=[]
        label_list=[]
        for l in bounding_boxes:
            boxes = l.boxes
            
            for box in boxes:
                
                boundingbox_list.append(box.xyxy[0])
                conf_list.append(box.conf[0])
                label_list.append(box.cls[0])
        
        
        return boundingbox_list,conf_list,label_list 

    def get_image_bb(self,image):
        bb_list,conf_list,label_list = self.get_boundingbox(image)
        for box in bb_list:
            image = cv2.rectangle(image,(box[0],box[1]),(box[2],box[3]),3)

        return image



    

    


    