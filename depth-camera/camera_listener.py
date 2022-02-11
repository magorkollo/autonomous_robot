#!/usr/bin/env/python

# first launch the camera to be able to get information back:
# roslaunch realsense2_camera rs_camera.launch

import rospy
from sensor_msgs.msg import CameraInfo
import time
import pickle

def listen_to_camera(data):
    global camera_info
    camera_info = data

def listener():
    rospy.init_node('listener', anonymous = True)
    rospy.Subscriber("/camera/depth/camera_info", CameraInfo, listen_to_camera)

if __name__ == '__main__':
    t_end = time.time() + 0.3
    while time.time() < t_end:
        listener()
    print(camera_info)
    file_name = "camera_file.txt"
    camera_file = open(file_name, "wb")

    # the data is saved using pickle, therefore when we are
    # reading the data back one sould use pickle again
    pickle.dump(camera_info, camera_file)
    camera_file.close
