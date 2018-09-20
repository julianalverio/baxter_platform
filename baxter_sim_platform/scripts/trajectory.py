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

# This is just a helper class provided by ROS
class Trajectory(object):
  def __init__(self, limb='left'):
      ns = 'robot/limb/' + limb + '/'
      self._client = actionlib.SimpleActionClient(
          ns + "follow_joint_trajectory",
          FollowJointTrajectoryAction,
      )
      self._goal = FollowJointTrajectoryGoal()
      self._goal_time_tolerance = rospy.Time(0.1)
      self._goal.goal_time_tolerance = self._goal_time_tolerance
      server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
      if not server_up:
          rospy.logerr("Timed out waiting for Joint Trajectory"
                       " Action Server to connect. Start the action server"
                       " before running example.")
          rospy.signal_shutdown("Timed out waiting for Action Server")
          sys.exit(1)
      self.clear(limb)

  def add_point(self, positions, time):
      point = JointTrajectoryPoint()
      point.positions = copy.copy(positions)
      point.time_from_start = rospy.Duration(time)
      self._goal.trajectory.points.append(point)

  def start(self):
      self._goal.trajectory.header.stamp = rospy.Time.now()
      self._client.send_goal(self._goal)

  def stop(self):
      self._client.cancel_goal()

  def wait(self, timeout=15.0):
      self._client.wait_for_result(timeout=rospy.Duration(timeout))

  def result(self):
      return self._client.get_result()

  def clear(self, limb):
      self._goal = FollowJointTrajectoryGoal()
      self._goal.goal_time_tolerance = self._goal_time_tolerance
      self._goal.trajectory.joint_names = [limb + '_' + joint for joint in \
          ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]

