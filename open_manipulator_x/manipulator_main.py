from manipulator_controller import OpenManipulatorX
import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
import math
import message_filters
from std_msgs.msg import Float64MultiArray
import time
#from moveit_commander.conversions import pose_to_list

# x,y,z offset values to transform from camera to joint
camera_arm_offset = [0.070, 0, 0.052]
object_list = []
number_objects = 0
id_list = []
ALL = False
EMPTY = False
#coordinates_sub = rospy.Subscriber("/coordinates_of_detections", Float64MultiArray)

def callb(data):
  global object_list
  global id_list
  global ALL
  if not ALL:
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    if data.data:
      number_objects = data.data[0]
      obj_id = data.data[1]
      if obj_id not in id_list:
        id_list.append(obj_id)
        new_list = [data.data[2], data.data[3], data.data[4], data.data[5]]
        object_list.append(new_list)
        if (len(id_list) == number_objects):
          ALL = True
  else:
    pass

def listener():
  coordinates_sub = rospy.Subscriber("/coordinates_of_detections", Float64MultiArray, callb)
  if ALL:
    coordinates_sub.unregister()


def degree_rad(deg):
  rad = (deg * pi)/180
  return rad

def showcase(open_manipulator_x):
    print("go to the next arm position")
    raw_input()
    angles = [degree_rad(0), degree_rad(-9), degree_rad(-22), degree_rad(78)]
    open_manipulator_x.go_to_joint_state(angles)
    print("go to the next arm position")
    raw_input()
    angles = [0,0,0,0]
    open_manipulator_x.go_to_joint_state(angles)
    print("go to the next arm position")
    raw_input()
    angles = [degree_rad(-90), degree_rad(1), degree_rad(1), degree_rad(-3)]
    open_manipulator_x.go_to_joint_state(angles)
    print("go to the next arm position")
    raw_input()
    angles = [degree_rad(0), degree_rad(-90), degree_rad(70), degree_rad(35)]
    open_manipulator_x.go_to_joint_state(angles)
    raw_input()
    #angles = [0 ,1.02 ,-0.907 ,-0.174]
    # open_manipulator_x.gripper_open()
    #angles = [0 ,-1.50 ,0.64 , 1.48]
    print("primary showcase is done")


def main():
  try:
    open_manipulator_x = OpenManipulatorX()
    current_joint_angles = open_manipulator_x.get_joint_angles()
    # angles = [degree_rad(0), degree_rad(-90), degree_rad(70), degree_rad(35)]
    # print(angles)
    # open_manipulator_x.go_to_joint_state(angles)
    # rospy.sleep(6)
    coordinates_sub = rospy.Subscriber("/coordinates_of_detections", Float64MultiArray, callb)
    while True:
      if ALL:
        rospy.loginfo("GOT ALL OF THEM SHuTDOWN NEEDED")
        print(object_list)
        coordinates_sub.unregister()
        break
      else:
        listener()
        print("WAITING FOR DATA", ALL)
      if EMPTY:
        rospy.loginfo("EMPTY, NO OBJECT DETECTED")
        break
      rospy.sleep(1)
    print("Manipulator path planning starts:")
    print(object_list)

    #open_manipulator_x.gripper_close()
    if len(object_list) > 0:
      open_manipulator_x.gripper_open()
      rospy.sleep(1)
      for i in range(len(object_list)):
        coord0 = object_list[i][0]
        coord1 = object_list[i][1]
        coord2 = object_list[i][2]
        dist = object_list[i][3]
        print("COORD0 before IF = ", coord0)
        if coord0 > 0:
          coord0 = -coord0
          coord0 = coord0 - 0.008
          coord2 = coord2 + 0.04
          coord1 = -0.02
        else:
          coord0 = abs(coord0) - 0.018
          coord2 = coord2 + 0.075
          coord1 = -0.05
        print(coord0)
        coords = [coord2, coord0,  -0.003]
        print("======COORDS======== ", coords)
        #coords = [coord2+0.04, coord1-0.05, -0.02]
        try:
          g_angle = pi/10
          angles = open_manipulator_x.inverse_kinematics(coords, g_angle)
          open_manipulator_x.go_to_joint_state(angles)
          rospy.sleep(8)
          open_manipulator_x.gripper_close()
          rospy.sleep(2)
          angles = [degree_rad(0), degree_rad(-57), degree_rad(17), degree_rad(40)]
          open_manipulator_x.go_to_joint_state(angles)
          rospy.sleep(8)
          angles = [degree_rad(-90), degree_rad(1), degree_rad(1), degree_rad(-3)]
          open_manipulator_x.go_to_joint_state(angles)
          rospy.sleep(8)
          open_manipulator_x.gripper_open()
          rospy.sleep(2)
          angles = [degree_rad(0), degree_rad(-90), degree_rad(70), degree_rad(35)]
          open_manipulator_x.go_to_joint_state(angles)
          rospy.sleep(8)
          print("SUCCEEDED")
          print(coords)
        except ValueError:
          print("not succedeed, math domain error...")
          print(coords)
          pass
          rospy.sleep(3)

    print("DONE")
    # if len(current_joint_angles) > 0 :
    #   comparation = all(joint == 0 for joint in current_joint_angles)
    # if comparation :
    #   print("all elements are zero, it will probably not work")
    # else :
    #   current_coordinates = open_manipulator_x.forward_kinematics(current_joint_angles)
    #   open_manipulator_x.gripper_open()
    #   print("go to predetermined coordinates ")
    #   raw_input()
    #   coords = [0.382-0.010, 0.135-0.095, 0.0017]
    #   g_angle = pi/12
    #   angles = open_manipulator_x.inverse_kinematics(coords, g_angle)
    #   print(angles)
    #   open_manipulator_x.go_to_joint_state(angles)
    #   raw_input()
    #   open_manipulator_x.gripper_close()
    #   print("go to predetermined angles of the joints")
    #   raw_input()
    #   angles = [0 ,1.02 ,-0.907 ,-0.174]
    #   #angles = [0,0,0,0]
    #   open_manipulator_x.go_to_joint_state(angles)


    
    #print(current_coordinates)

    
    # print("go to predetermined coordinates ")
    # raw_input()
    # coords = [0.305+0.070, 0.0, 0.127-0.052]
    # g_angle = 0
    # angles = open_manipulator_x.inverse_kinematics(coords, g_angle)
    # print(angles)
    # open_manipulator_x.go_to_joint_state(angles)
    # raw_input()
    # angles = [degree_rad(0), degree_rad(-90), degree_rad(70), degree_rad(35)]
    # open_manipulator_x.go_to_joint_state(angles)

    # print("fully open gripper")
    # raw_input()
    # open_manipulator_x.gripper_open()

    # print("fully close gripper")
    # raw_input()
    # open_manipulator_x.gripper_close()

    # print("open gripper with a given percentage")
    # raw_input()
    # open_percentage = 50 # give a percentage you would like the gripper to be opened
    # open_manipulator_x.gripper_precision(open_percentage)

    # print("fully close gripper")
    # raw_input()
    # open_manipulator_x.gripper_close()
    
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
