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


from trajectory import *
from robot_controller import *
from scene_controller import *
from utils import *

class Manager(object):
  def __init__(self):
    rospy.init_node("baxter_platform_manager")
    rospy.wait_for_message("/robot/sim/started", Empty)
    self.robot_controller = RobotController()
    self.scene_controller = SceneController()

  def shutdown(self):
    robot_controller.shutdown()
    scene_controller.shutdown()





def main():
  manager = Manager()
  rospy.on_shutdown(manager.shutdown())
  starting_angles = {'left_w0': 0.6699952259595108,
                   'left_w1': 1.030009435085784,
                   'left_w2': -0.4999997247485215,
                   'left_e0': -1.189968899785275,
                   'left_e1': 1.9400238130755056,
                   'left_s0': -0.08000397926829805,
                   'left_s1': -0.9999781166910306}
  import pdb; pdb.set_trace()
  Model(name='table', shape='box', roll=0., pitch=0., yaw=0., restitution_coeff=0., size_x=.7, size_y=1.5, size_z=.7, x=.4, y=0.1, z=0., mass=5000, color='Grey', mu1=1, mu2=1, reference_frame='')

  trajectory = manager.robot_controller.generateTrajectoryPose()
  manager.robot_controller.followTrajectoryWithIK(trajectory)

  Model(name='example_block', shape='box', size_x=0.1, size_y=0.2, size_z=0.3, x=0.1, y=0.2, z=0.3, mass=0.1, color='Red', mu1=1000, mu2=2000, restitution_coeff=0.5, roll=0.1, pitch=0.2, yaw=0.3)
  spawnGazeboModels(models)
  import pdb; pdb.set_trace()



if __name__ == '__main__':
  sys.exit(main())