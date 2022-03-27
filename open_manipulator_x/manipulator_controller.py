#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
import math

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class OpenManipulatorX(object):
  def __init__(self):
    super(OpenManipulatorX, self).__init__()
    # OpenManipulator initialization, based on the MoveIt!'s Python interface
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('manipulator_node', anonymous=True)
    arm = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_arm = "arm"
    move_group_arm = moveit_commander.MoveGroupCommander(group_arm)
    group_gripper = "gripper"
    move_group_gripper = moveit_commander.MoveGroupCommander(group_gripper)
    display_trajectory_publisher = rospy.Publisher('/move_group_arm/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)


    print("INFORMATION ABOUT THE ARM: ")
    planning_frame = move_group_arm.get_planning_frame()
    print("Planning frame: %s" % planning_frame)

    eef_link = move_group_arm.get_end_effector_link()
    print("End effector link: %s" % eef_link)

    planning_frame_gripper = move_group_gripper.get_planning_frame()
    print("GRIPPER FRAME: %s" % planning_frame_gripper)

    eef_link_gripper = move_group_gripper.get_end_effector_link()
    print("End effector link GRIPPER: %s" % eef_link_gripper)

    group_names = arm.get_group_names()
    print("Available Planning Groups:", arm.get_group_names())

    print("Printing arm state")
    print arm.get_current_state()

    self.box_name = ''
    self.arm = arm
    self.scene = scene
    self.position_gripper_closed = -0.01
    self.position_gripper_opened = 0.019
    self.move_group_arm = move_group_arm
    self.move_group_gripper = move_group_gripper
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names


  def go_to_joint_state(self, angles):

    joint_goal = self.move_group_arm.get_current_joint_values()
    joint_goal[0] = angles[0]
    joint_goal[1] = angles[1]
    joint_goal[2] = angles[2]
    joint_goal[3] = angles[3]

    self.move_group_arm.go(joint_goal, wait=True)
    self.move_group_arm.stop()
    current_joints = self.move_group_arm.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def gripper_close(self):

    joint_goal = self.move_group_gripper.get_current_joint_values()
    joint_goal[0] = self.position_gripper_closed

    self.move_group_gripper.go(joint_goal, wait=True)
    self.move_group_gripper.stop()

    current_joints = self.move_group_gripper.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def gripper_open(self):

    joint_goal = self.move_group_gripper.get_current_joint_values()
    joint_goal[0] = self.position_gripper_opened

    self.move_group_gripper.go(joint_goal, wait=True)
    self.move_group_gripper.stop()

    current_joints = self.move_group_gripper.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def gripper_precision(self, open_percentage):
    
    value_to_open = ((open_percentage *(abs(self.position_gripper_closed) + self.position_gripper_opened)) / 100)
    joint_goal = self.move_group_gripper.get_current_joint_values()
    joint_goal[0] = self.position_gripper_closed + value_to_open

    self.move_group_gripper.go(joint_goal, wait=True)
    self.move_group_gripper.stop()

    current_joints = self.move_group_gripper.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def display_trajectory(self, plan):
    arm = self.arm

    display_trajectory_publisher = self.display_trajectory_publisher
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = self.arm.get_current_state()

    display_trajectory.trajectory.append(plan)
    # Publish
    self.display_trajectory_publisher.publish(display_trajectory)

  def get_joint_angles(self):
    return self.move_group_arm.get_current_joint_values()

  def forward_kinematics(self, joint_angles):
      '''
      Calculates the TCP coordinates from the joint angles
      :param joint_angles: list, joint angles [j0, j1, j2, j3]
      :return: list, the list of TCP coordinates
      '''
      # link lengths
      l1 = 0.128
      l2 = 0.024
      l3 = 0.124
      l4 = 0.126

      # offsets
      x_offset = 0.012
      z_offset = 0.0595 + 0.017

      x = x_offset + (l1 * math.sin(joint_angles[1]) + l2 * math.cos(joint_angles[1]) + l3 * math.cos(joint_angles[1] + joint_angles[2]) + l4 * math.cos(joint_angles[1] + joint_angles[2] + joint_angles[3])) * math.cos(joint_angles[0])
      y = (l1 * math.sin(joint_angles[1]) + l2 * math.cos(joint_angles[1]) + l3 * math.cos(joint_angles[1] + joint_angles[2]) + l4 * math.cos(joint_angles[1] + joint_angles[2] + joint_angles[3])) * math.sin(joint_angles[0])
      z = z_offset + l1 * math.cos(joint_angles[1]) - l2 * math.sin(joint_angles[1]) - l3 * math.sin(joint_angles[1] + joint_angles[2]) - l4 * math.sin(joint_angles[1] + joint_angles[2] + joint_angles[3])

      return [x,y,z]

  def inverse_kinematics(self, coords, gripper_angle = 0):
    '''
    Calculates the joint angles according to the desired TCP coordinate and gripper angle
    :param coords: list, desired [X, Y, Z] TCP coordinates
    :param gripper_angle: float, gripper angle in woorld coordinate system (0 = horizontal, pi/2 = vertical)
    :return: list, the list of joint angles, including the 2 gripper fingers
    '''
    # link lengths
    l1 = 0.128
    l2 = 0.024
    l1c = 0.13023 # ua_link = combined l1 - l2 length
    l3 = 0.124    # fa_link
    l4 = 0.126    # tcp_link

    # base offsets
    x_offset = 0.012
    z_offset = 0.0595 + 0.017

    # joint offsets due to combined l1 - l2
    j1_offset = math.atan(l2/l1)
    j2_offset = math.pi/2.0 + j1_offset # includes +90 degrees offset, too

    # default return list
    angles = [0,0,0,0]

    # Calculate the shoulder pan angle from x and y coordinates
    j0 = math.atan(coords[1]/(coords[0] - x_offset))

    # Re-calculate target coordinated to the wrist joint (x', y', z')
    x = coords[0] - x_offset - l4 * math.cos(j0) * math.cos(gripper_angle)
    y = coords[1] - l4 * math.sin(j0) * math.cos(gripper_angle)
    z = coords[2] - z_offset + math.sin(gripper_angle) * l4

    # Solve the problem in 2D using x" and z'
    x = math.sqrt(y*y + x*x)

    # Let's calculate auxiliary lengths and angles
    c = math.sqrt(x*x + z*z)
    alpha = math.asin(z/c)
    beta = math.pi - alpha
    # Apply law of cosines
    gamma = math.acos((l1c*l1c + c*c - l3*l3)/(2*c*l1c))

    j1 = math.pi/2.0 - alpha - gamma - j1_offset
    j2 = math.acos((l1c*l1c + l3*l3 - c*c)/(2*l1c*l3)) - j2_offset
    delta = math.pi - j2 - gamma - j2_offset

    j3 = math.pi + gripper_angle - beta - delta

    angles[0] = j0
    angles[1] = j1
    angles[2] = -j2
    angles[3] = j3

    return angles

