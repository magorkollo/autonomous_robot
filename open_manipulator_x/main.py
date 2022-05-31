from manipulator_controller import OpenManipulatorX
import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
import math
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

# x,y,z offset values to transform from camera to joint
camera_arm_offset = [0.070, 0, 0.052]

def degree_rad(deg):
  rad = (deg * pi)/180
  return rad

def main():
  try:
    open_manipulator_x = OpenManipulatorX()
    current_joint_angles = open_manipulator_x.get_joint_angles()
    comparation = False
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

    
    #open_manipulator_x.gripper_close()
    
    #open_manipulator_x.gripper_open()
    # print("go to predetermined coordinates ")
    # raw_input()
    # coords = [0.344+0.070, 0.0, 0.122-0.052]
    # g_angle = 0
    # angles = open_manipulator_x.inverse_kinematics(coords, g_angle)
    # print(angles)
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
    
    #print("go to predetermined angles of the joints")
    print("go to the next arm position")
    raw_input()
    angles = [degree_rad(0), degree_rad(-9), degree_rad(-22), degree_rad(78)]
    open_manipulator_x.go_to_joint_state(angles)
    print("go to the next arm position")
    raw_input()
    # angles = [degree_rad(97), degree_rad(-73), degree_rad(50), degree_rad(23)]
    # open_manipulator_x.go_to_joint_state(angles)
    # raw_input()
    angles = [0,0,0,0]
    open_manipulator_x.go_to_joint_state(angles)
    print("go to the next arm position")
    raw_input()
    angles = [degree_rad(-151), degree_rad(-17), degree_rad(21), degree_rad(-3)]
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
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
