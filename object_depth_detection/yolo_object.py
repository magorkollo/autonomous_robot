import cv2
import numpy as np 
import argparse
import time
from depth_camera_control import *
import math



class YoloNet:
    def __init__(self):
        self.net = cv2.dnn.readNet("/home/jnano/autonomous_robot/object_depth_detection/yolo_dependencies/yolov3-tiny.weights", "yolo_dependencies/yolov3-tiny.cfg")
        self.classes = []
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
        with open("yolo_dependencies/coco.names", "r") as names:
            self.classes = [line.strip() for line in names.readlines()]
        self.output_layers = [layer_name for layer_name in self.net.getUnconnectedOutLayersNames()]
        self.colors = np.random.uniform(0, 255, size=(len(self.classes), 3))
        self.start_time = time.time() 
    
    def init_depth_camera(self):
        self.dc = DepthCamera()
    
    def start_webcam(self):
        video = cv2.VideoCapture(2)
        return video

    def display_blob(blob):
        for b in blob:
            for n, imgb in enumerate(b):
                cv2.imshow(str(n), imgb)

    def detect_objects(self, img):
        blob = cv2.dnn.blobFromImage(img, scalefactor=0.00392, size=(320, 320), mean=(0, 0, 0), swapRB=True, crop=False)
        self.net.setInput(blob)
        results = self.net.forward(self.output_layers)
        return blob, results
    
    def get_box_dimensions(self, results, height, width):
        boxes = []
        confs = []
        class_ids = []
        centers = []
        lefts = []
        rights = []
        for result in results:
            for detect in result:
                scores = detect[5:]
                class_id = np.argmax(scores)
                if class_id == 47:
                    conf = scores[class_id]
                    if conf > 0.25:
                        center_x = int(detect[0] * width)
                        center_y = int(detect[1] * height)
                        w = int(detect[2] * width)
                        h = int(detect[3] * height)
                        x = int(center_x - w/2)
                        y = int(center_y - h / 2)
                        boxes.append([x, y, w, h])
                        centers.append([center_x, center_y])
                        confs.append(float(conf))
                        class_ids.append(class_id)
        return boxes, confs, class_ids, centers


    def get_box_dimensions_depth(self, results, height, width):
        boxes = []
        confs = []
        class_ids = []
        centers = []
        lefts = []
        rights = []
        for result in results:
            for detect in result:
                scores = detect[5:]
                class_id = np.argmax(scores)
                if class_id == 47:
                    conf = scores[class_id]
                    if conf > 0.25:
                        center_x = int(detect[0] * width)
                        center_y = int(detect[1] * height)
                        w = int(detect[2] * width)
                        h = int(detect[3] * height)
                        x = int(center_x - w/2)
                        y = int(center_y - h / 2)
                        boxes.append([x, y, w, h])
                        centers.append([center_x, center_y])
                        lefts.append([int(x+10), int(y+h/2)])
                        rights.append([int(x+w-10), int(y+h/2)])
                        confs.append(float(conf))
                        class_ids.append(class_id)
        return boxes, confs, class_ids, centers, lefts, rights
    
    def calculate_2d_distance(self, point0, point1):
        sum_y = point0[0] - point1[0]
        sum_z = point0[1] - point1[1]
        distance_2d = math.sqrt(abs(pow(sum_y,2) - pow(sum_z,2)))
        return distance_2d

    def calculate_depth(self, left, right):
        distance_2d = 0
        depth_coord_left = self.dc.get_depth_coordinates(left[0], left[1])
        depth_coord_right = self.dc.get_depth_coordinates(right[0], right[1])
        left_deep = [depth_coord_left[0], depth_coord_left[1]]
        right_deep = [depth_coord_right[0], depth_coord_right[1]]
        distance_2d = calculate_2d_distance(left_deep, right_deep)
        return distance_2d

    def draw_labels(self, boxes, confs, class_ids, centers, img): 
        indexes = cv2.dnn.NMSBoxes(boxes, confs, 0.5, 0.4)
        font = cv2.FONT_HERSHEY_PLAIN
        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                center = centers[i]
                label = str(self.classes[class_ids[i]])
                color = self.colors[i]
                cv2.rectangle(img, (x,y), (x+w, y+h), color, 2)
                cv2.circle(img, center, 4, (0, 0, 255))
                cv2.putText(img, label, (x, y - 5), font, 1, color, 1)
        cv2.imshow("Image", img)
    
    def draw_labels_depth(self, boxes, confs, class_ids, centers, lefts, rights, img): 
        indexes = cv2.dnn.NMSBoxes(boxes, confs, 0.5, 0.4)
        font = cv2.FONT_HERSHEY_PLAIN
        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                label = str(self.classes[class_ids[i]])
                color = self.colors[i]
                center = centers[i]
                left = lefts[i]
                right = rights[i]
                depth_coords_center = self.dc.ssd_get_depth_coordinates(center[0], center[1])
                depth_coord_left = self.dc.ssd_get_depth_coordinates(left[0], left[1])
                depth_coord_right = self.dc.ssd_get_depth_coordinates(right[0], right[1])
                print("dpth = ", depth_coords_center[2])
                print("x = ", depth_coords_center[0])
                print("y = ", depth_coords_center[1])
                left_deep = [depth_coord_left[0], depth_coord_left[1]]
                right_deep = [depth_coord_right[0], depth_coord_right[1]]
                distance_2d = self.calculate_2d_distance(left_deep, right_deep)
                cv2.putText(img, "dpth: {0:.3f}".format(depth_coords_center[2]), (center[0], center[1] - 65), cv2.FONT_HERSHEY_PLAIN, 0.9, (255, 255, 255), 2)
                cv2.putText(img, "x: {0:.3f}".format(depth_coords_center[0]), (center[0], center[1] - 75), cv2.FONT_HERSHEY_PLAIN, 0.9, (255, 255, 255), 2)
                cv2.putText(img, "y: {0:.3f}".format(depth_coords_center[1]), (center[0], center[1] - 85), cv2.FONT_HERSHEY_PLAIN, 0.9, (255, 255, 255), 2)
                cv2.rectangle(img, (x,y), (x+w, y+h), color, 2)
                cv2.circle(img, tuple(center), 4, (255, 255, 255))
                cv2.circle(img, tuple(left), 4, (255, 0, 255))
                cv2.circle(img, tuple(right), 4, (120, 120, 0))
                cv2.line(img, tuple(left), tuple(right), (120, 120, 0), 2)
                cv2.putText(img, "dist: {0:.3f}".format(distance_2d), (left[0], left[1] - 10), cv2.FONT_HERSHEY_PLAIN, 1.2, (255, 255, 255), 2)
                cv2.putText(img, label, (x, y - 5), font, 1, color, 1)
        cv2.imshow("YOLO - Tiny v3", img)

    def calculate_depth(self, boxes, confs, class_ids, centers, lefts, rights, img): 
        indexes = cv2.dnn.NMSBoxes(boxes, confs, 0.5, 0.4)
        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                label = str(self.classes[class_ids[i]])
                color = self.colors[i]
                center = centers[i]
                left = lefts[i]
                right = rights[i]
                depth_coords_center = self.dc.ssd_get_depth_coordinates(center[0], center[1])
                depth_coord_left = self.dc.ssd_get_depth_coordinates(left[0], left[1])
                depth_coord_right = self.dc.ssd_get_depth_coordinates(right[0], right[1])
                print("object = ",label)
                print("dpth = ", depth_coords_center[2])
                print("x = ", depth_coords_center[0])
                print("y = ", depth_coords_center[1])
                left_deep = [depth_coord_left[0], depth_coord_left[1]]
                right_deep = [depth_coord_right[0], depth_coord_right[1]]
                distance_2d = self.calculate_2d_distance(left_deep, right_deep)
        cv2.imshow("YOLO - Tiny v3", img)


    def normal_detection(self):
        cap = self.start_webcam()
        self.start = time.time()
        while True:
            self.start_time = time.time()
            _, frame = cap.read()
            height, width, channels = frame.shape
            blob, outputs = self.detect_objects(frame)
            boxes, confs, class_ids, centers= self.get_box_dimensions(outputs, height, width)
            self.draw_labels(boxes, confs, class_ids, centers, frame)
            print("FPS: ", 1.0 / (time.time() - self.start_time))
            key = cv2.waitKey(1)
            if key == 27:
                break
            if (time.time() - self.start) > 10:
                print("10 SEC OVER")
                break
        cap.release()
    
    def depth_headless(self):
        self.init_depth_camera()
        color_frame, depth_frame_distance = self.dc.ssd_get_frame()
        self.start = time.time()
        while True:
            self.start_time = time.time()
            color_frame, depth_frame_distance = self.dc.ssd_get_frame()
            height, width, channels = color_frame.shape
            blob, outputs = self.detect_objects(color_frame)
            boxes, confs, class_ids, centers, lefts, rights = self.get_box_dimensions_depth(outputs, height, width)
            self.calculate_depth(boxes, confs, class_ids, lefts, lefts, rights, color_frame)
            print("FPS: ", 1.0 / (time.time() - self.start_time)) 
            key = cv2.waitKey(1)
            if key == 27:
                break
            if (time.time() - self.start) > 10:
                print("10 SEC OVER")
                break
        self.dc.release()

    def depth_display(self):
        self.init_depth_camera()
        self.start = time.time()
        while True:
            self.start_time = time.time() 
            color_frame, depth_frame_distance = self.dc.ssd_get_frame()
            height, width, channels = color_frame.shape
            blob, outputs = self.detect_objects(color_frame)
            boxes, confs, class_ids, centers, lefts, rights = self.get_box_dimensions_depth(outputs, height, width)
            self.draw_labels_depth(boxes, confs, class_ids, lefts, lefts, rights, color_frame)
            print("FPS P: ", 1.0 / (time.time() - self.start_time)) 
            key = cv2.waitKey(1)
            if key == 27:
                break
            if (time.time() - self.start) > 10:
                print("10 SEC OVER")
                break
        self.dc.release()
        #cap.release()
    

if __name__ == '__main__':
    yolo = YoloNet()
    yolo.normal_detection()
<<<<<<< HEAD
    cv2.destroyAllWindows()
=======
    cv2.destroyAllWindows()
>>>>>>> 241db205fd0d29b68cab1317dabed967bdd43ad6
