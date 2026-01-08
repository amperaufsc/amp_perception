import depthai as dai
import cv2
from fs_msgs.msg import Track
from fs_msgs.msg import Cone
from cv_bridge import CvBridge
import numpy as np

bridge = CvBridge()

nnBlobPath = '/home/jetson/ws/src/as_amp/perception/position_estimation/best_nano_openvino_2022.1_6shave.blob'

class depthai_detection: 
    def __init__(self):
        
        self.labelMap = ["blue_cone", "large_orange_cone", "orange_cone", "unknown_cone", "yellow_cone"]  

        # Create DepthAI pipeline
        pipeline = dai.Pipeline()

        # Define sources and outputs
        spatialDetectionNetwork = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
        rgbLeft = pipeline.create(dai.node.ColorCamera)
        rgbRight = pipeline.create(dai.node.ColorCamera)
        stereo = pipeline.create(dai.node.StereoDepth)

        xoutNN = pipeline.create(dai.node.XLinkOut)
        xoutDepth = pipeline.create(dai.node.XLinkOut)
        
        xoutLeft = pipeline.create(dai.node.XLinkOut)
        xoutRight = pipeline.create(dai.node.XLinkOut)

        xoutNN.setStreamName("detections")
        xoutDepth.setStreamName("depth")
        
        xoutLeft.setStreamName('left')
        xoutRight.setStreamName('right')

        # Camera properties
        rgbLeft.setPreviewSize(640,640)
        rgbLeft.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1200_P)
        rgbLeft.setCamera("left")
        rgbLeft.setIspScale(2,3)
        rgbLeft.setFps(40)
        rgbLeft.setInterleaved(False)
        rgbRight.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1200_P)
        rgbRight.setCamera("right")
        rgbRight.setIspScale(2,3)
        rgbRight.setFps(40)

        # StereoDepth configuration
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_B)
        stereo.setExtendedDisparity(True)
        stereo.setSubpixel(True)

        # YOLOv8 spatial detection network
        spatialDetectionNetwork.setBlobPath(nnBlobPath)
        spatialDetectionNetwork.setConfidenceThreshold(0.7)
        spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
        spatialDetectionNetwork.setNumInferenceThreads(3)

        # YOLO specific parameters
        spatialDetectionNetwork.setNumClasses(5)
        spatialDetectionNetwork.setCoordinateSize(4)
        spatialDetectionNetwork.setIouThreshold(0.7)
        
        # Linking 
        rgbLeft.isp.link(stereo.left)
        rgbRight.isp.link(stereo.right)
        rgbLeft.preview.link(spatialDetectionNetwork.input)
        stereo.depth.link(spatialDetectionNetwork.inputDepth)
        spatialDetectionNetwork.passthrough.link(xoutLeft.input)
        spatialDetectionNetwork.out.link(xoutNN.input)
        spatialDetectionNetwork.passthroughDepth.link(xoutDepth.input)
        
        stereo.disparity.link(xoutDepth.input)
        rgbRight.isp.link(xoutRight.input)

        # Create a device
        self.device = dai.Device(pipeline)

    def get_image_with_intrinsics(self):
        
        calibData = self.device.readCalibration()
        intrinsic_matrix = np.array(calibData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A,640,640))
        distortion_matrix = calibData.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A)

        return intrinsic_matrix, distortion_matrix
    
    def process_data(self):
        
        # Output queues
        self.detectionQueue = self.device.getOutputQueue(name="detections", maxSize=4, blocking=False)
        self.depthQueue = self.device.getOutputQueue(name="depth", maxSize=4, blocking=False)

        qLeft = self.device.getOutputQueue(name="left", maxSize=4, blocking=False).get()
        qRight = self.device.getOutputQueue(name="right", maxSize=4, blocking=False).get()

        inDetections = self.detectionQueue.get()
        inDepth = self.depthQueue.get()

        inLeftFrame = qLeft.getCvFrame() if qLeft else None
        inRightFrame = qRight.getCvFrame() if qRight else None
        

        # Get depth frame
        depthFrame = inDepth.getFrame()
        depthFrameColor = cv2.normalize(depthFrame, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8UC1)
        depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_JET)

        # Process detections
        detections = inDetections.detections
        
        cone_list = []
        track = Track()

        for detection in detections:
    
            # Label and confidence
            label = self.labelMap[detection.label] if detection.label < len(self.labelMap) else "Unknown"
            spatialCoords = detection.spatialCoordinates

            # Publish 3D position (X, Y, Z) as a Point message
            position_msg = Cone()
            position_msg.location.x = spatialCoords.x / 1000 # X position in m
            position_msg.location.y = spatialCoords.y / 1000 # Y position in m
            position_msg.location.z = spatialCoords.z / 1000 # Z position in m

            if label == 'blue_cone':
                    position_msg.color=0
            elif label == 'yellow_cone':
                    position_msg.color=1
            elif label == 'large_orange_cone':
                    position_msg.color=2

            cone_list.append(position_msg)
        
        track.track = cone_list
        
        return depthFrame, inLeftFrame, inRightFrame, track
    
