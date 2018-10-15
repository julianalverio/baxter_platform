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
from gazebo_msgs.msg import ContactsState, ModelStates
import csv


from trajectory import *
from robot_controller import *
from scene_controller import *
from utils import *

class Manager(object):
  def __init__(self, moveit=True):
    rospy.init_node("baxter_platform_manager")
    rospy.wait_for_message("/robot/sim/started", Empty)
    # self.robot_controller = RobotController(moveit=moveit, tolerance=0.1)
    self.robot_controller = RobotController(moveit=moveit)
    self.scene_controller = SceneController()

  def shutdown(self):
    self.robot_controller.shutdown()
    self.scene_controller.deleteAllModels()



def main():
  ######## An simple example usage of the Baxter Platform  ######


  manager = Manager(moveit=False)
  rospy.on_shutdown(manager.shutdown)

  hover_position = {
  'left_w0': 0.740299201640411, 
  'left_w1': 0.9680624138988025, 
  'left_w2': -0.7136691281471845, 
  'left_e0': -0.7285205258543106, 
  'left_e1': 1.5080107556603004, 
  'left_s0': -0.4963593155154979, 
  'left_s1': -0.622152160192126}

  manager.scene_controller.makeModel(name='table', shape='box', roll=0., pitch=0., yaw=0., restitution_coeff=0., size_x=.7, size_y=1.5, size_z=.7, x=.8, y=0., z=.35, mass=5000, ambient_r=0.1, ambient_g=0.1, ambient_b=0.1, ambient_a=0.1, mu1=1, mu2=1, reference_frame='')
  manager.scene_controller.makeModel(name='testObject', shape='box', size_x=0.1, size_y=0.1, size_z=0.1, x=0.8, y=0.3, z=0.75, mass=20000, mu1=1000, mu2=2000, restitution_coeff=0.5, roll=0.0, pitch=0.0, yaw=0.0, ambient_r=0, ambient_g=1, ambient_b=0, ambient_a=1, diffuse_r=0, diffuse_g=1, diffuse_b=0, diffuse_a=1)
  manager.scene_controller.spawnAllModels()

  manager.robot_controller.moveToStart()
  manager.robot_controller.followTrajectoryFromJointAngles([hover_position])
  print("Sleeping")
  rospy.sleep(5.)


if __name__ == '__main__':
  sys.exit(main())