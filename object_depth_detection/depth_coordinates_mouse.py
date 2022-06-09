import cv2
#import pyrealsense2.pyrealsense2 as rs
import pyrealsense2 as rs
from depth_camera_control import *
import numpy as np

# random initialization of the point we are looking for
point = (1, 1)

def show_distance(event, x, y, args, params):
    global point
    point = (x, y)

# Initialize Camera Intel Realsense
dc = DepthCamera()
intrinsics = dc.get_intrinsics()

# create mouse event
cv2.namedWindow("Mouse frame")
cv2.setMouseCallback("Mouse frame", show_distance)

while True:
    ret, depth, color_frame, points, depth_frame_distance = dc.get_frame()

    # Show distance for a specific point
    cv2.circle(color_frame, point, 4, (0, 0, 255))
    distance = dc.get_distance(point[0], point[1])
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

    cv2.imshow("Mouse frame", color_frame)
    key = cv2.waitKey(1)
    if key == 27:
        break
