#!/usr/bin/env python

import argparse
import struct
import sys
import copy
import os
import re
import random
import numpy as np

import rospy
import rospkg

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from std_msgs.msg import (
    Header,
    Empty,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from baxter_core_msgs.msg import (
    JointCommand,
)

import baxter_interface
from baxter_interface import CHECK_VERSION

import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

import genpy

from std_srvs.srv import Empty as placeholder

from gazebo_msgs.srv import GetLinkState
from gazebo_msgs.msg import ContactsState
import csv

#for contact sensors
def contactCallback(data):
  if data.states:
    print 'Contact detected!'
    print data.states


def nonantipodalGrasp():
  print 'Showing: Nonantipodal Grasp of Rectangular Prism.'

  robo_controller = RobotController()
  robo_controller._gripper.set_holding_force(100)
  rospy.on_shutdown(cleanUp)

  print 'setting up camera...'
  externalCamera()

  robo_controller.gripper_open()

  print 'moving to start position...'
  pre_grip_angles = {'left_w0': 0.713473354262754, 'left_w1': 1.014095801262804, 'left_w2': -0.7107767620135959, 'left_e0': -0.598464939148772, 'left_e1': 0.9698857738523418, 'left_s0': -0.8576164362879198, 'left_s1': -0.2443509381144592}
  robo_controller._limb.move_to_joint_positions(pre_grip_angles)

  ##work table
  models = list()
  models.append(Model(name='table', shape='block', roll=0., pitch=0., yaw=0., restitution_coeff=0., size_x=.7, size_y=1.5, size_z=.7, x=.4, y=0.1, z=0., mass=5000, color='Grey', mu1=1, mu2=1, reference_frame=''))
  
  print 'generating table'
  loadGazeboModels(models)

  models = []
  models.append(Model(name='test', shape='block', yaw=0.5, size_x=0.1, size_y=0.018, size_z=0.1, x=0.75/2, y=-0.00, z=0.3, mass=1, color='Yellow', mu1=1000, mu2=1000, reference_frame='world'))

  print 'generating test object'
  loadGazeboModels(models)

  rospy.Subscriber("/l_side_r_finger_contact_sensor_state", ContactsState, contactCallback)
  rospy.Subscriber("/l_side_l_finger_contact_sensor_state", ContactsState, contactCallback)
  rospy.Subscriber("/r_side_r_finger_contact_sensor_state", ContactsState, contactCallback)
  rospy.Subscriber("/r_side_l_finger_contact_sensor_state", ContactsState, contactCallback)

  print 'Gripping now.'
  robo_controller.gripper_close()
  starting_overhead_angles = {'left_w0': 0.590693567655256, 'left_w1': 1.059736120576616, 'left_w2': -0.7595985440689625, 'left_e0': -1.1059240339618084, 'left_e1': 1.8919082513633318, 'left_s0': -0.1417684774660966, 'left_s1': -1.0218518327712416}
  robo_controller._limb.move_to_joint_positions(starting_overhead_angles)

  print 'Done. Hanging now.'
  while not rospy.is_shutdown():
    pass
  return 0


def pushObjectOnTable():
  print 'Pushing Rectangular Prism Into Table.'

  robo_controller = RobotController()
  robo_controller._gripper.set_holding_force(100)
  rospy.on_shutdown(cleanUp)

  print 'setting up camera...'
  externalCamera()


  robo_controller.gripper_open()
  print 'moving to start position...'
  pre_grip_angles = {'left_w0': 0.713473354262754, 'left_w1': 1.014095801262804, 'left_w2': -0.7107767620135959, 'left_e0': -0.598464939148772, 'left_e1': 0.9698857738523418, 'left_s0': -0.8576164362879198, 'left_s1': -0.2443509381144592}
  robo_controller._limb.move_to_joint_positions(pre_grip_angles)


  ##work table
  models = list()
  models.append(Model(name='table', shape='block', roll=0., pitch=0., yaw=0., restitution_coeff=0., size_x=.7, size_y=1.5, size_z=.7, x=.4, y=0.1, z=0., mass=5000, color='Grey', mu1=1, mu2=1, reference_frame=''))
  
  print 'generating table'
  loadGazeboModels(models)

  models = []
  models.append(Model(name='test', shape='block', yaw=0.0, size_x=0.1, size_y=0.018, size_z=0.1, x=0.75/2, y=-0.00, z=0.3, mass=1, color='Yellow', mu1=1000, mu2=1000, reference_frame='world'))

  print 'generating test object'
  loadGazeboModels(models)

  rospy.Subscriber("/l_side_r_finger_contact_sensor_state", ContactsState, contactCallback)
  rospy.Subscriber("/l_side_l_finger_contact_sensor_state", ContactsState, contactCallback)
  rospy.Subscriber("/r_side_r_finger_contact_sensor_state", ContactsState, contactCallback)
  rospy.Subscriber("/r_side_l_finger_contact_sensor_state", ContactsState, contactCallback)

  print 'Gripping now.'
  robo_controller.gripper_close()

  current_pose = robo_controller._limb.endpoint_pose()
  next_pose = Pose()
  next_pose.position.x = current_pose['position'].x
  next_pose.position.y = current_pose['position'].y
  next_pose.position.z = current_pose['position'].z + 0.1
  next_pose.orientation = current_pose['orientation']

  angles = robo_controller.solveIK(next_pose)
  robo_controller._limb.move_to_joint_positions(angles)

  current_pose = robo_controller._limb.endpoint_pose()
  next_pose = Pose()
  next_pose.position.x = current_pose['position'].x
  next_pose.position.y = current_pose['position'].y
  next_pose.position.z = current_pose['position'].z - 0.2
  next_pose.orientation = current_pose['orientation']

  angles = robo_controller.solveIK(next_pose)
  robo_controller._limb.move_to_joint_positions(angles)

  #it should not get past this point.

  print 'Done. Hanging now.'
  while not rospy.is_shutdown():
    pass
  return 0


def pushHandOnTable():
  print 'Crashing Hand Into Table Top'

  robo_controller = RobotController()
  robo_controller._gripper.set_holding_force(100)
  rospy.on_shutdown(cleanUp)

  print 'setting up camera...'
  externalCamera()

  print 'moving to start position...'

  robo_controller.gripper_close()

  pre_grip_angles = {'left_w0': 0.713473354262754, 'left_w1': 1.014095801262804, 'left_w2': -0.7107767620135959, 'left_e0': -0.598464939148772, 'left_e1': 0.9698857738523418, 'left_s0': -0.8576164362879198, 'left_s1': -0.2443509381144592}
  robo_controller._limb.move_to_joint_positions(pre_grip_angles)

  ##work table
  models = list()
  models.append(Model(name='table', shape='block', roll=0., pitch=0., yaw=0., restitution_coeff=0., size_x=.7, size_y=1.5, size_z=.7, x=.4, y=0.1, z=0., mass=5000, color='Grey', mu1=1, mu2=1, reference_frame=''))
  
  print 'generating table'
  loadGazeboModels(models)

  rospy.Subscriber("/l_side_r_finger_contact_sensor_state", ContactsState, contactCallback)
  rospy.Subscriber("/l_side_l_finger_contact_sensor_state", ContactsState, contactCallback)
  rospy.Subscriber("/r_side_r_finger_contact_sensor_state", ContactsState, contactCallback)
  rospy.Subscriber("/r_side_l_finger_contact_sensor_state", ContactsState, contactCallback)

  current_pose = robo_controller._limb.endpoint_pose()
  next_pose = Pose()
  next_pose.position.x = current_pose['position'].x
  next_pose.position.y = current_pose['position'].y
  next_pose.position.z = current_pose['position'].z + 0.1
  next_pose.orientation = current_pose['orientation']

  angles = robo_controller.solveIK(next_pose)
  robo_controller._limb.move_to_joint_positions(angles)

  current_pose = robo_controller._limb.endpoint_pose()
  next_pose = Pose()
  next_pose.position.x = current_pose['position'].x
  next_pose.position.y = current_pose['position'].y
  next_pose.position.z = current_pose['position'].z - 0.2
  next_pose.orientation = current_pose['orientation']

  angles = robo_controller.solveIK(next_pose)
  robo_controller._limb.move_to_joint_positions(angles)

  #It should not get past this point.

  print 'Done. Hanging now.'
  while not rospy.is_shutdown():
    pass
  return 0


def grabCorners():
  print 'Grabbing Corners of Rectagular Prism.'

  robo_controller = RobotController()
  robo_controller._gripper.set_holding_force(100)
  rospy.on_shutdown(cleanUp)

  print 'setting up camera...'
  externalCamera()

  print 'moving to start position...'

  
  robo_controller.gripper_open()

  pre_grip_angles = {'left_w0': 0.713473354262754, 'left_w1': 1.014095801262804, 'left_w2': -0.7107767620135959, 'left_e0': -0.598464939148772, 'left_e1': 0.9698857738523418, 'left_s0': -0.8576164362879198, 'left_s1': -0.2443509381144592}
  robo_controller._limb.move_to_joint_positions(pre_grip_angles)

  ##work table
  models = list()
  models.append(Model(name='table', shape='block', roll=0., pitch=0., yaw=0., restitution_coeff=0., size_x=.7, size_y=1.5, size_z=.7, x=.4, y=0.1, z=0., mass=5000, color='Grey', mu1=1, mu2=1, reference_frame=''))
  
  print 'generating table'
  loadGazeboModels(models)

  models = []
  models.append(Model(name='test', shape='block', yaw=np.pi/4., size_x=0.02, size_y=0.018, size_z=0.1, x=0.75/2, y=-0.00, z=0.3, mass=1, color='Yellow', mu1=1000, mu2=1000, reference_frame='world'))

  print 'generating test object'
  loadGazeboModels(models)

  rospy.Subscriber("/l_side_r_finger_contact_sensor_state", ContactsState, contactCallback)
  rospy.Subscriber("/l_side_l_finger_contact_sensor_state", ContactsState, contactCallback)
  rospy.Subscriber("/r_side_r_finger_contact_sensor_state", ContactsState, contactCallback)
  rospy.Subscriber("/r_side_l_finger_contact_sensor_state", ContactsState, contactCallback)

  print 'Gripping now.'
  robo_controller.gripper_close()

  current_pose = robo_controller._limb.endpoint_pose()
  next_pose = Pose()
  next_pose.position.x = current_pose['position'].x
  next_pose.position.y = current_pose['position'].y
  next_pose.position.z = current_pose['position'].z + 0.1
  next_pose.orientation = current_pose['orientation']

  angles = robo_controller.solveIK(next_pose)
  robo_controller._limb.move_to_joint_positions(angles)

  current_pose = robo_controller._limb.endpoint_pose()
  next_pose = Pose()
  next_pose.position.x = current_pose['position'].x
  next_pose.position.y = current_pose['position'].y
  next_pose.position.z = current_pose['position'].z - 0.2
  next_pose.orientation = current_pose['orientation']

  angles = robo_controller.solveIK(next_pose)
  robo_controller._limb.move_to_joint_positions(angles)

  #It should not get past this point.

  print 'Done. Hanging now.'
  while not rospy.is_shutdown():
    pass
  return 0


def manipulateCylinder():
  print 'Manipulating Cylinder.'

  robo_controller = RobotController()
  robo_controller._gripper.set_holding_force(100)
  rospy.on_shutdown(cleanUp)

  print 'setting up camera...'
  externalCamera()

  print 'moving to start position...'
  robo_controller.gripper_open()

  pre_grip_angles = {'left_w0': 0.713473354262754, 'left_w1': 1.014095801262804, 'left_w2': -0.7107767620135959, 'left_e0': -0.598464939148772, 'left_e1': 0.9698857738523418, 'left_s0': -0.8576164362879198, 'left_s1': -0.2443509381144592}
  robo_controller._limb.move_to_joint_positions(pre_grip_angles)

  ##work table
  models = list()
  models.append(Model(name='table', shape='block', roll=0., pitch=0., yaw=0., restitution_coeff=0., size_x=.7, size_y=1.5, size_z=.7, x=.4, y=0.1, z=0., mass=5000, color='Grey', mu1=1, mu2=1, reference_frame=''))
  
  print 'generating table'
  loadGazeboModels(models)

  models = []
  models.append(Model(name='test', shape='cylinder', yaw=0., size_r=0.018, size_z=0.12, x=0.75/2, y=-0.00, z=0.3, mass=1, color='Yellow', mu1=1000, mu2=1000, reference_frame='world'))

  print 'generating test object'
  loadGazeboModels(models)

  rospy.Subscriber("/l_side_r_finger_contact_sensor_state", ContactsState, contactCallback)
  rospy.Subscriber("/l_side_l_finger_contact_sensor_state", ContactsState, contactCallback)
  rospy.Subscriber("/r_side_r_finger_contact_sensor_state", ContactsState, contactCallback)
  rospy.Subscriber("/r_side_l_finger_contact_sensor_state", ContactsState, contactCallback)

  print 'Gripping now.'
  robo_controller.gripper_close()

  current_pose = robo_controller._limb.endpoint_pose()
  next_pose = Pose()
  next_pose.position.x = current_pose['position'].x
  next_pose.position.y = current_pose['position'].y
  next_pose.position.z = current_pose['position'].z + 0.1
  next_pose.orientation = current_pose['orientation']

  angles = robo_controller.solveIK(next_pose)
  robo_controller._limb.move_to_joint_positions(angles)

  current_pose = robo_controller._limb.endpoint_pose()
  next_pose = Pose()
  next_pose.position.x = current_pose['position'].x
  next_pose.position.y = current_pose['position'].y
  next_pose.position.z = current_pose['position'].z - 0.2
  next_pose.orientation = current_pose['orientation']

  angles = robo_controller.solveIK(next_pose)
  robo_controller._limb.move_to_joint_positions(angles)

  print 'Done. Hanging now.'
  while not rospy.is_shutdown():
    pass
  return 0



def manipulateSphere():
  print 'Manipulating Sphere.'

  robo_controller = RobotController()
  robo_controller._gripper.set_holding_force(100)
  rospy.on_shutdown(cleanUp)

  print 'setting up camera...'
  externalCamera()

  robo_controller.gripper_open()

  print 'moving to start position...'
  pre_grip_angles = {'left_w0': 0.713473354262754, 'left_w1': 1.014095801262804, 'left_w2': -0.7107767620135959, 'left_e0': -0.598464939148772, 'left_e1': 0.9698857738523418, 'left_s0': -0.8576164362879198, 'left_s1': -0.2443509381144592}
  robo_controller._limb.move_to_joint_positions(pre_grip_angles)

  ##work table
  models = list()
  models.append(Model(name='table', shape='block', roll=0., pitch=0., yaw=0., restitution_coeff=0., size_x=.7, size_y=1.5, size_z=.7, x=.4, y=0.1, z=0., mass=5000, color='Grey', mu1=1, mu2=1, reference_frame=''))
  
  print 'generating table'
  loadGazeboModels(models)

  models = []
  models.append(Model(name='test', shape='sphere', yaw=0., size_r=0.02, x=0.75/2, y=0.005, z=0.3, mass=1, color='Yellow', mu1=1000, mu2=1000, reference_frame='world'))

  print 'generating test object'
  loadGazeboModels(models)

  current_pose = robo_controller._limb.endpoint_pose()
  next_pose = Pose()
  next_pose.position.x = current_pose['position'].x
  next_pose.position.y = current_pose['position'].y
  next_pose.position.z = current_pose['position'].z - 0.05
  next_pose.orientation = current_pose['orientation']

  print 'doing IK for next movement...'
  next_angles = robo_controller.solveIK(next_pose)
  print 'moving to grasp'
  robo_controller._limb.move_to_joint_positions(next_angles)

  rospy.Subscriber("/l_side_r_finger_contact_sensor_state", ContactsState, contactCallback)
  rospy.Subscriber("/l_side_l_finger_contact_sensor_state", ContactsState, contactCallback)
  rospy.Subscriber("/r_side_r_finger_contact_sensor_state", ContactsState, contactCallback)
  rospy.Subscriber("/r_side_l_finger_contact_sensor_state", ContactsState, contactCallback)

  print 'Gripping now.'
  robo_controller.gripper_close()

  current_pose = robo_controller._limb.endpoint_pose()
  next_pose = Pose()
  next_pose.position.x = current_pose['position'].x
  next_pose.position.y = current_pose['position'].y
  next_pose.position.z = current_pose['position'].z + 0.1
  next_pose.orientation = current_pose['orientation']

  angles = robo_controller.solveIK(next_pose)
  robo_controller._limb.move_to_joint_positions(angles)

  current_pose = robo_controller._limb.endpoint_pose()
  next_pose = Pose()
  next_pose.position.x = current_pose['position'].x
  next_pose.position.y = current_pose['position'].y
  next_pose.position.z = current_pose['position'].z - 0.2
  next_pose.orientation = current_pose['orientation']

  angles = robo_controller.solveIK(next_pose)
  robo_controller._limb.move_to_joint_positions(angles)

  #It should not get past this point.

  print 'Done. Hanging now.'
  while not rospy.is_shutdown():
    pass

  return 0


def hitHandOnTableSide():
  print 'Colliding with Side of Table.'
  

  robo_controller = RobotController()
  robo_controller._gripper.set_holding_force(100)
  rospy.on_shutdown(cleanUp)

  print 'setting up camera...'
  externalCamera()

  robo_controller.gripper_close()

  print 'moving to start position...'
  pre_grip_angles = {'left_w0': 0.713473354262754, 'left_w1': 1.014095801262804, 'left_w2': -0.7107767620135959, 'left_e0': -0.598464939148772, 'left_e1': 0.9698857738523418, 'left_s0': -0.8576164362879198, 'left_s1': -0.2443509381144592}
  robo_controller._limb.move_to_joint_positions(pre_grip_angles)

  ##work table
  models = list()
  models.append(Model(name='table', shape='block', roll=0., pitch=0., yaw=0., restitution_coeff=0., size_x=.7, size_y=1.5, size_z=.85, x=.4, y=0.45, z=0., mass=5000, color='Grey', mu1=1, mu2=1, reference_frame=''))
  
  print 'generating table'
  loadGazeboModels(models)

  current_pose = robo_controller._limb.endpoint_pose()
  next_pose = Pose()
  next_pose.position.x = current_pose['position'].x
  next_pose.position.y = current_pose['position'].y + 0.3
  next_pose.position.z = current_pose['position'].z
  next_pose.orientation = current_pose['orientation']

  next_angles = robo_controller.solveIK(next_pose)
  robo_controller._limb.move_to_joint_positions(next_angles)

  #It shouldn't get past this point.

  print 'Done. Hanging now.'
  while not rospy.is_shutdown():
    pass
  return 0


def edgeCases():
  rospy.init_node("ik_pick_and_place_demo")
  rospy.wait_for_message("/robot/sim/started", Empty)

  # nonantipodalGrasp()
  # pushObjectOnTable()
  # grabCorners()
  # manipulateSphere()
  manipulateCylinder()
  pushHandOnTable()
  hitHandOnTableSide()