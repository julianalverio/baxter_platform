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
  manager = Manager(moveit=False)
  rospy.on_shutdown(manager.shutdown)
  position1 = {'left_w0': 0.6699952259595108,
                   'left_w1': 1.030009435085784,
                   'left_w2': -0.4999997247485215,
                   'left_e0': -1.189968899785275,
                   'left_e1': 1.9400238130755056,
                   'left_s0': -0.08000397926829805,
                   'left_s1': -0.9999781166910306}
  position2 = {'left_w0': 0.713473354262754, 
               'left_w1': 1.014095801262804, 
               'left_w2': -0.7107767620135959, 
               'left_e0': -0.598464939148772, 
               'left_e1': 0.9698857738523418, 
               'left_s0': -0.8576164362879198, 
               'left_s1': -0.2443509381144592}

  point = Point(x=0.7, y=0.15, z=-0.12)
  overhead_orientation = Quaternion(
                             x=-0.0249590815779,
                             y=0.999649402929,
                             z=0.00737916180073,
                             w=0.00486450832011)
  pose = Pose(position=point,
        orientation=overhead_orientation)
  angles = manager.robot_controller.solveIK(pose)
  # position0 = [0.19184445920935822, 1.0470001074930844, 0.006743514728764666, 0.49792895371651014, -0.1843767142558015, 0.025892859061044327, -0.01438656665986393]
  # position1 = [0.1699952259595108, 1.030009435085784, 0.4999997247485215,0.49189968899785275, 1.9400238130755056, -0.08000397926829805, -0.01499781166910306]
  # position2 = [0.713473354262754, 1.014095801262804, -0.7107767620135959, 0.598464939148772, 0.9698857738523418, -0.8576164362879198, -0.2443509381144592]
  # trajectory = [position0, position1, position2]
  # print("I'm trying now!")
  # manager.robot_controller.followMoveItTrajectoryWithJointAngles(trajectory)
  # print("I'm done")
  #Z position for table of height 0.1 need be -0.59
  # manager.scene_controller.makeModel(name='table', shape='box', roll=0., pitch=0., yaw=0., restitution_coeff=0., size_x=.7, size_y=1.5, size_z=.7, x=.8, y=0., z=.35, mass=5000, ambient_r=0.1, ambient_g=0.1, ambient_b=0.1, ambient_a=0.1, mu1=1, mu2=1, reference_frame='')
  manager.scene_controller.makeModel(name='table', shape='box', roll=0., pitch=0., yaw=0., restitution_coeff=0., size_x=.1, size_y=1.5, size_z=.7, x=.7, y=0., z=.35, mass=5000, ambient_r=0.1, ambient_g=0.1, ambient_b=0.1, ambient_a=0.1, mu1=1, mu2=1, reference_frame='')
  manager.scene_controller.makeModel(name='testObject', shape='box', size_x=0.1, size_y=0.1, size_z=0.1, x=0.8, y=0.3, z=0.75, mass=20000, mu1=1000, mu2=2000, restitution_coeff=0.5, roll=0.1, pitch=0.2, yaw=0.3, ambient_r=0, ambient_g=1, ambient_b=0, ambient_a=1, diffuse_r=0, diffuse_g=1, diffuse_b=0, diffuse_a=1)
  manager.scene_controller.spawnAllModels(moveit=True)

  manager.robot_controller.moveToStart()
  manager.robot_controller.followTrajectoryFromJointAngles([angles])

  print("I'm done!")
  import pdb; pdb.set_trace()




if __name__ == '__main__':
  sys.exit(main())