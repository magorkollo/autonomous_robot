#!/usr/bin/env python
#export DISPLAY=:0
#export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1
import jetson.inference
import jetson.utils
import cv2
from depth_camera_control import *
import numpy as np
import time
import math

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

def calculate_2d_distance(point0, point1):
    sum_y = point0[0] - point1[0]
    sum_z = point0[1] - point1[1]
    distance_2d = math.sqrt(abs(pow(sum_y,2) - pow(sum_z,2)))
    return distance_2d


def calculate_depth_central_line(CenterY, Left, Right):
    distance_2d = 0
    depth_coord_left = dc.ssd_get_depth_coordinates(int(Left), int(CenterY))
    depth_coord_right = dc.ssd_get_depth_coordinates(int(Right), int(CenterY))
    left_deep = [depth_coord_left[0], depth_coord_left[1]]
    right_deep = [depth_coord_right[0], depth_coord_right[1]]
    distance_2d = calculate_2d_distance(left_deep, right_deep)
    return distance_2d


counter = 0 
start_time = time.time()
x = 1 # displays the frame rate every 1 second
counter = 0
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
        center = (int(detection.Center[0]), int(detection.Center[1]))
        cv2.circle(cv_img, (int(detection.Right), center[1]), 5, (255,0, 0), -1)
        cv2.circle(cv_img, (int(detection.Left), center[1]), 5, (255, 0, 0), -1)
        cv2.circle(cv_img, (int(detection.Center[0]), center[1]), 5, (0, 0, 255), -1) 
        depth_coords_center = dc.ssd_get_depth_coordinates(center[0], center[1])
        distance_2d = calculate_depth_central_line(center[1], int(detection.Left), int(detection.Right))
        # cv2.putText(img, "dpth: {0:.3f}".format(depth_coords_center[2]), (center[0], center[1] - 65), cv2.FONT_HERSHEY_PLAIN, 0.9, (255, 255, 255), 2)
        # cv2.putText(img, "x: {0:.3f}".format(depth_coords_center[0]), (center[0], center[1] - 75), cv2.FONT_HERSHEY_PLAIN, 0.9, (255, 255, 255), 2)
        # cv2.putText(img, "y: {0:.3f}".format(depth_coords_center[1]), (center[0], center[1] - 85), cv2.FONT_HERSHEY_PLAIN, 0.9, (255, 255, 255), 2)
        cv2.line(cv_img, (int(detection.Left), int(detection.Center[1])), (int(detection.Right), int(detection.Center[1])), (0, 255, 0), 1)
        # cv2.putText(img, "dist: {0:.3f}".format(distance_2d), (center[0], int(detection.Left) - 10), cv2.FONT_HERSHEY_PLAIN, 1.2, (255, 255, 255), 2)
        print(distance_2d)
        print(depth_coords_center)

    counter+=1
    if (time.time() - start_time) > x :
        print("FPS: ", counter / (time.time() - start_time))
        counter = 0
        start_time = time.time() 
    if testing:
        img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2RGBA).astype(np.float32)
        img = jetson.utils.cudaFromNumpy(img)
        display.Render(img)

