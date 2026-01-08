import depthai as dai
import cv2
from cv_bridge import CvBridge
import numpy as np

bridge = CvBridge()

class depthai_camera_setup: 
    def __init__(self):
        
        self.labelMap = ["blue_cone", "large_orange_cone", "orange_cone", "unknown_cone", "yellow_cone"]  

        # Create DepthAI pipeline
        pipeline = dai.Pipeline()

        # Define sources and outputs
        rgbLeft = pipeline.create(dai.node.ColorCamera)
        rgbRight = pipeline.create(dai.node.ColorCamera)
        stereo = pipeline.create(dai.node.StereoDepth)
        
        xoutDepth = pipeline.create(dai.node.XLinkOut)
        xoutLeft = pipeline.create(dai.node.XLinkOut)
        xoutRight = pipeline.create(dai.node.XLinkOut)

        xoutDepth.setStreamName("depth")
        xoutLeft.setStreamName('left')
        xoutRight.setStreamName('right')

        # Camera properties
        rgbLeft.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1200_P)
        rgbLeft.setCamera("left")
        rgbLeft.setIspScale(2,3)
        rgbRight.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1200_P)
        rgbRight.setCamera("right")
        rgbRight.setIspScale(2,3)
        # StereoDepth configuration
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setExtendedDisparity(True)
        stereo.setSubpixel(True)

        # Linking 
        rgbLeft.isp.link(stereo.left)
        rgbRight.isp.link(stereo.right)

        stereo.disparity.link(xoutDepth.input)
        rgbRight.isp.link(xoutRight.input)
        rgbLeft.isp.link(xoutLeft.input)

        # Create a device
        self.device = dai.Device(pipeline)

    def get_image_with_intrinsics(self):
        
        calibData = self.device.readCalibration()
        intrinsic_matrix = np.array(calibData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A,640,640))
        distortion_matrix = calibData.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A)

        return intrinsic_matrix, distortion_matrix
    
    ###

    def process_data(self):

        # Output queues
        self.depthQueue = self.device.getOutputQueue(name="depth", maxSize=4, blocking=False).get()
        qLeft = self.device.getOutputQueue(name="left", maxSize=4, blocking=False).get()
        qRight = self.device.getOutputQueue(name="right", maxSize=4, blocking=False).get()

        inLeftFrame = qLeft.getCvFrame() if qLeft else None
        inRightFrame = qRight.getCvFrame() if qRight else None
        
        # Get depth frame
        depthFrame = self.depthQueue.getFrame()
        depthFrameColor = cv2.normalize(depthFrame, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8UC1)
        depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_JET)

        return depthFrame, inLeftFrame, inRightFrame
    
    