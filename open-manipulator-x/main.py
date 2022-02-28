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

def main():
  try:
    open_manipulator_x = OpenManipulatorX()

    print("go to predetermined coordinates ")
    raw_input()
    coords = [0.075, -0.2, 0.195]
    angles = open_manipulator_x.inverse_kinematics(coords)
    open_manipulator_x.go_to_joint_state(angles)

    print("fully open gripper")
    raw_input()
    open_manipulator_x.gripper_open()

    print("fully close gripper")
    raw_input()
    open_manipulator_x.gripper_close()

    print("open gripper with a given percentage")
    raw_input()
    open_percentage = 60 # give a percentage you would like the gripper to be opened
    open_manipulator_x.gripper_precision(open_percentage)

    print("fully close gripper")
    raw_input()
    open_manipulator_x.gripper_close()

    print("go to predetermined angles of the joints")
    raw_input()
    angles = [pi/2 ,0 ,0 ,0]
    open_manipulator_x.go_to_joint_state(angles)



    print("primary showcase is done")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
