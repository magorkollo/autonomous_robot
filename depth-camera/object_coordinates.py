import cv2
import pyrealsense2.pyrealsense2 as rs
# import pyrealsense2 as rs
from depth_camera_control import *
import numpy as np
import pickle

import jetson.inference
import jetson.utils

net = jetson.inference.detectNet("ssd-inception-v2", threshold=0.35)
#camera = jetson.utils.videoSource("/dev/video0")      # '/dev/video0' for V4L2
display = jetson.utils.videoOutput("display://0") # 'my_video.mp4' for file

# random initialization of the point we are looking for
point = (1, 1)
IDs = (53, 55)

def show_distance(event, x, y, args, params):
    global point
    point = (x, y)

# Initialize Camera Intel Realsense
dc = DepthCamera()
profile = dc.get_profile()
print(profile)

depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
print("The depth scale is: ", depth_scale)

with open('camera_file.txt', 'rb') as camera_file:
    global camera_info
    camera_info = pickle.load(camera_file)

intrinsics = rs.intrinsics()
intrinsics.width = camera_info.width
intrinsics.height = camera_info.height
intrinsics.ppx = camera_info.K[2]
intrinsics.ppy = camera_info.K[5]
intrinsics.fx = camera_info.K[0]
intrinsics.fy = camera_info.K[4]
intrinsics.model = rs.distortion.none
intrinsics.coeffs = [i for i in camera_info.D]

# create mouse event
cv2.namedWindow("Color frame")
#cv2.setMouseCallback("Color frame", show_distance)


def listener():
    rospy.init_node('listener', anonymous = True)
    rospy.Subscriber("/camera/depth/camera_info", CameraInfo, listen_to_camera)
    rospy.spin()

while True:
    ret, depth, color_frame, points, depth_frame_distance = dc.get_frame()

    # Show distance for a specific point

    bgr_img = jetson.utils.cudaFromNumpy(color_frame, isBGR=True)
    rgb_img = jetson.utils.cudaAllocMapped(width = bgr_img.width, 
    height = bgr_img.height, format="rgb8")
    jetson.utils.cudaConvertColor(bgr_img, rgb_img)
    detections = net.Detect(rgb_img)
    display.Render(rgb_img)
    display.SetStatus("Object Detection | Network {:.0f} FPS".format(net.GetNetworkFPS()))

    for object_detected in detections:
        if object_detected.ClassID in IDs:
            point = (int(object_detected.Center[0]), int(object_detected.Center[1]))
            print(point)
            cv2.circle(color_frame, point, 4, (255, 255, 0))
            distance = depth_frame_distance.get_distance(point[0], point[1])
            result = rs.rs2_deproject_pixel_to_point(intrinsics, [point[0], point[1]], distance)
            x = result[2]
            y = result[0]
            z = result[1]
            print("x = ", x)
            print("y = ", y)
            print("z = ", z)
            cv2.putText(color_frame, "distance mm: {0:.3f}".format(x), (point[0], point[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)
            cv2.putText(color_frame, "y: {0:.3f}".format(y), (point[0], point[1] - 45), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)
            cv2.putText(color_frame, "z: {0:.3f}".format(z), (point[0], point[1] - 70), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)
            

    cv2.imshow("Color frame", color_frame)
    key = cv2.waitKey(1)
    if key == 27:
        break
