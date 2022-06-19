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
import pickle

neural_network = None
display = None
camera = None
public = None
testing = True

class SSDNet():
    def __init__(self):
        self.neural_network = jetson.inference.detectNet(argv=['--model=ssd_dependencies/ssd-mobilenet.onnx','--labels=ssd_dependencies/labels.txt', '--input_blob=input_0', '--output-cvg=scores', '--output-bbox=boxes'], threshold = 0.2)
        self.detection_list = []

    
    def init_depth_camera(self):
        self.dc = DepthCamera()

    def init_display(self):
        self.display = jetson.utils.videoOutput("")

    
    def init_camera(self):
        self.camera = jetson.utils.videoSource("/dev/video2") # '/dev/video2' for D435i - RGB data

    def calculate_2d_distance(self, point0, point1):
        sum_y = point0[0] - point1[0]
        sum_z = point0[1] - point1[1]
        distance_2d = math.sqrt(abs(pow(sum_y,2) - pow(sum_z,2)))
        return distance_2d

    def calculate_depth_central_line(self, CenterY, Left, Right):
        distance_2d = 0
        depth_coord_left = self.dc.ssd_get_depth_coordinates(int(Left), int(CenterY))
        depth_coord_right = self.dc.ssd_get_depth_coordinates(int(Right), int(CenterY))
        left_deep = [depth_coord_left[0], depth_coord_left[1]]
        right_deep = [depth_coord_right[0], depth_coord_right[1]]
        distance_2d = self.calculate_2d_distance(left_deep, right_deep)
        return distance_2d

    def color_to_cuda(self, col_img):
        img = cv2.cvtColor(col_img, cv2.COLOR_BGR2RGB)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2RGBA).astype(np.float32)
        cuda_mem = jetson.utils.cudaFromNumpy(img)
        return cuda_mem

    def cuda_to_color(self, cuda_mem):
        img = jetson.utils.cudaToNumpy(cuda_mem, 1280, 720, 4)
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2RGB).astype(np.uint8)
        cv_img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        jetson.utils.cudaDeviceSynchronize()
        return cv_img

    def depth_display(self):
        self.init_depth_camera()
        self.init_display()
        self.start = time.time()
        while True:
            self.start_time = time.time()
            color_image, depth_frame_distance = self.dc.ssd_get_frame()
            cuda_mem = self.color_to_cuda(color_image)
            detections = self.neural_network.Detect(cuda_mem)
            cv_img = self.cuda_to_color(cuda_mem)
            counter = 0
            #print("Object Detection | Network {:.0f} FPS".format(self.neural_network.GetNetworkFPS()))
            for detection in detections:
                local_list = []
                counter = counter + 1
                name_obj = self.neural_network.GetClassDesc(detection.ClassID)+str(counter) # name of the class
                print(name_obj)
                center = (int(detection.Center[0]), int(detection.Center[1]))
                cv2.circle(cv_img, (int(detection.Right), center[1]), 5, (255,0, 0), -1)
                cv2.circle(cv_img, (int(detection.Left), center[1]), 5, (255, 0, 0), -1)
                cv2.circle(cv_img, (int(detection.Center[0]), center[1]), 5, (0, 0, 255), -1) 
                depth_coords_center = self.dc.ssd_get_depth_coordinates(center[0], center[1])
                distance_2d = self.calculate_depth_central_line(center[1], int(detection.Left), int(detection.Right))
                cv2.putText(cv_img, "dpth: {0:.3f}".format(depth_coords_center[2]), (center[0], center[1] - 75), cv2.FONT_HERSHEY_PLAIN, 0.9, (255, 255, 255), 2)
                cv2.putText(cv_img, "x: {0:.3f}".format(depth_coords_center[0]), (center[0], center[1] - 85), cv2.FONT_HERSHEY_PLAIN, 0.9, (255, 255, 255), 2)
                cv2.putText(cv_img, "y: {0:.3f}".format(depth_coords_center[1]), (center[0], center[1] - 95), cv2.FONT_HERSHEY_PLAIN, 0.9, (255, 255, 255), 2)
                cv2.putText(cv_img, "name: " + name_obj, (center[0], center[1] - 115), cv2.FONT_HERSHEY_PLAIN, 0.9, (255, 255, 255), 2)
                cv2.line(cv_img, (int(detection.Left), int(detection.Center[1])), (int(detection.Right), int(detection.Center[1])), (0, 255, 0), 1)
                cv2.putText(cv_img, "dist: {0:.3f}".format(distance_2d), (center[0], center[1] - 10), cv2.FONT_HERSHEY_PLAIN, 1.2, (255, 255, 255), 2)
                print(depth_coords_center)
                local_list = [name_obj, depth_coords_center, distance_2d]
                if all(i != 0 for i in depth_coords_center):
                    self.detection_list.append(local_list)
            if (len(self.detection_list) > 12):
                print("enough detections registered")
                break
            print("FPS: ", 1.0 / (time.time() - self.start_time))
            if (time.time() - self.start) > 12:
                print("10 SEC OVER")
                break
            if testing:
                cuda_mem = self.color_to_cuda(cv_img)
                self.display.Render(cuda_mem)
        self.dc.release()

    def depth_headless(self):
        self.init_depth_camera()
        self.start = time.time()
        while True:
            self.start_time = time.time() 
            color_image, depth_frame_distance = self.dc.ssd_get_frame()
            cuda_mem = self.color_to_cuda(color_image)
            detections = self.neural_network.Detect(cuda_mem)
            counter = 0
            for detection in detections:
                local_list = []
                counter = counter + 1
                name_obj = self.neural_network.GetClassDesc(detection.ClassID)+str(counter) # name of the class
                print(name_obj)
                center = (int(detection.Center[0]), int(detection.Center[1]))
                depth_coords_center = self.dc.ssd_get_depth_coordinates(center[0], center[1])
                distance_2d = self.calculate_depth_central_line(center[1], int(detection.Left), int(detection.Right))
                print(distance_2d)
                print(depth_coords_center)
                print(depth_coords_center)
                local_list = [name_obj, depth_coords_center, distance_2d]
                if all(i != 0 for i in depth_coords_center):
                    self.detection_list.append(local_list)
                if (len(self.detection_list) > 12):
                    print("enough detections registered")
                    break
            print("FPS: ", 1.0 / (time.time() - self.start_time))
            if (time.time() - self.start) > 5:
                print("5 SEC OVER")
                break 
        self.dc.release()

    def get_detect_list(self):
        print(self.detection_list)
        return self.detection_list

    def normal(self):
        self.init_camera()
        self.init_display()
        self.start = time.time()
        while True:
            self.start_time = time.time() 
            img = self.camera.Capture()
            detections = self.neural_network.Detect(img)
            print("FPS: ", 1.0 / (time.time() - self.start_time)) 
            if (time.time() - self.start) > 10:
                print("10 SEC OVER")
                break
            self.display.Render(img)

def process_list(object_list):
    pass

if __name__ == '__main__':
    ssd = SSDNet()
    ssd.depth_headless()
    object_list = ssd.get_detect_list()
    with open('object_list.txt', 'wb') as fp:
        pickle.dump(object_list, fp)
    cv2.destroyAllWindows()