#!/usr/bin/env python
#export DISPLAY=:0
#export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1
import jetson.inference
import jetson.utils
import cv2
from depth_camera_control import *
import numpy as np

neural_network = None
display = None
camera = None
public = None
testing = True

dc = DepthCamera()



def init_network():
    global neural_network
    global camera
    global display
    #camera = jetson.utils.videoSource("/dev/video2") # '/dev/video2' for D435i - RGB data
    display = jetson.utils.videoOutput("") # 'my_video.mp4' for file
    neural_network = jetson.inference.detectNet(argv=['--model=ssd_dependencies/ssd-mobilenet.onnx','--labels=ssd_dependencies/labels.txt', '--input_blob=input_0', '--output-cvg=scores', '--output-bbox=boxes'], threshold = 0.2)

init_network()

while display.IsStreaming():
    color_image, depth_frame_distance = dc.ssd_get_frame()
    img = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
    img = cv2.cvtColor(img, cv2.COLOR_RGB2RGBA).astype(np.float32)
    rgb_img = jetson.utils.cudaFromNumpy(img)
    detections = neural_network.Detect(rgb_img)
    img = jetson.utils.cudaToNumpy(rgb_img, 1280, 720, 4)
    img = cv2.cvtColor(img, cv2.COLOR_RGBA2RGB).astype(np.uint8)
    cv_img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    jetson.utils.cudaDeviceSynchronize()
    print("Object Detection | Network {:.0f} FPS".format(neural_network.GetNetworkFPS()))
    for detection in detections:
        print(neural_network.GetClassDesc(detection.ClassID))
        cv2.circle(cv_img, (int(detection.Center[0]), int(detection.Center[1])), 5, (0,0, 255), -1)
        print(dc.ssd_get_depth_coordinates(int(detection.Center[0]), int(detection.Center[1])))
        # print(int(detection.Center))ret, depth, color_frame, points, depth_frame_distance 
        # print(int(detection.Left))
        # print(int(detection.Right))
        


    if testing:
        img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2RGBA).astype(np.float32)
        img = jetson.utils.cudaFromNumpy(img)
        display.Render(img)

