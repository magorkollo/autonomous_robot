import cv2
import numpy as np 
import argparse
import time
from depth_camera_control import *

class YoloNet:
    def __init__(self):
        self.net = cv2.dnn.readNet("yolo_dependencies/yolov3.weights", "yolo_dependencies/yolov3.cfg")
        self.classes = []
        with open("yolo_dependencies/coco.names", "r") as names:
            self.classes = [line.strip() for line in names.readlines()]
        self.output_layers = [layer_name for layer_name in self.net.getUnconnectedOutLayersNames()]
        self.colors = np.random.uniform(0, 255, size=(len(self.classes), 3))
    
    def load_image(img_path):
        img = cv2.imread(img_path)
        img = cv2.resize(img, None, fx=0.4, fy=0.4)
        height, width, channels = img.shape
        return img, height, width, channels
    
    def start_webcam(self):
        video = cv2.VideoCapture(0)
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
        for result in results:
            for detect in result:
                scores = detect[5:]
                class_id = np.argmax(scores)
                conf = scores[class_id]
                if conf > 0.25:
                    center_x = int(detect[0] * width)
                    center_y = int(detect[1] * height)
                    w = int(detect[2] * width)
                    h = int(detect[3] * height)
                    x = int(center_x - w/2)
                    y = int(center_y - h / 2)
                    boxes.append([x, y, w, h])
                    confs.append(float(conf))
                    class_ids.append(class_id)
        return boxes, confs, class_ids
    
    def draw_labels(self, boxes, confs, class_ids, img): 
        indexes = cv2.dnn.NMSBoxes(boxes, confs, 0.5, 0.4)
        font = cv2.FONT_HERSHEY_PLAIN
        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                label = str(self.classes[class_ids[i]])
                color = self.colors[i]
                cv2.rectangle(img, (x,y), (x+w, y+h), color, 2)
                cv2.putText(img, label, (x, y - 5), font, 1, color, 1)
        cv2.imshow("Image", img)

    def image_detect(sefl, img_path): 
        #model, classes, colors, output_layers = load_yolo()
        image, height, width, channels = self.load_image(img_path)
        blob, outputs = self.detect_objects(image)
        boxes, confs, class_ids = self.get_box_dimensions(outputs, height, width)
        self.draw_labels(boxes, confs, class_ids , image)
        while True:
            key = cv2.waitKey(1)
            if key == 27:
                break

    def webcam_detect(self):
        #model, classes, colors, output_layers = load_yolo()
        cap = self.start_webcam()
        while True:
            _, frame = cap.read()
            height, width, channels = frame.shape
            blob, outputs = self.detect_objects(frame)
            boxes, confs, class_ids = self.get_box_dimensions(outputs, height, width)
            self.draw_labels(boxes, confs, class_ids, frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
        cap.release()
    
    

if __name__ == '__main__':
    yolo = YoloNet()
    yolo.webcam_detect()
    cv2.destroyAllWindows()