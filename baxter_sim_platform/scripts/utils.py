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

from robot_controller import RobotController
import rospkg


#samples_per = samples per trajectory execution (not actually executed)
def generateData(num_samples=1000, ending_angles_dict=None, samples_per=8):
  rospack = rospkg.RosPack()
  path = rospack.get_path('baxter_sim_platform') + '/scripts/noisydata.csv'
  writer = csv.writer(open(path, 'w+'))
  joints = ['left_w0', 'left_w1', 'left_w2', 'left_e0', 'left_e1', 'left_s0', 'left_s1']
  start = {'left_w0': 0.6699952259595108, 'left_w1': 1.030009435085784, 'left_w2': -0.4999997247485215,'left_e0': -1.189968899785275,'left_e1': 1.9400238130755056,'left_s0': -0.08000397926829805,'left_s1': -0.9999781166910306}
  start_angles = np.array([start[joint] for joint in joints])
  if not ending_angles_dict:
    ending_angles_dict = {'left_w0': 0.5956179611450151, 'left_w1': 0.9844394493019387, 'left_w2': -0.41598924558046574, 'left_e0': -1.2763027864959016, 'left_e1': 1.660696693321956, 'left_s0': -0.1343923972459911, 'left_s1': -0.9408210252355302}
  ending_angles = np.array([ending_angles_dict[joint] for joint in joints])
  for _ in xrange(num_samples):
    noise = np.random.uniform(-0.2, 0.2, 7)
    noisy_angles = ending_angles + noise
    deltas = noisy_angles - start_angles
    data_point = [start_angles]
    for sample_location in sorted(np.random.uniform(0, 1, samples_per)):
      sample = start_angles + deltas * sample_location
      data_point.append(sample.tolist())
    data_point.append(noisy_angles)
    writer.writerow(data_point)


#each cell in the csv is a list of angles indicating one poisition in a trajectory execution
#each row is a trajectory execution, so [eval(point) for point in row] is one trajectory execution
#the collection of trajectory executions constitutes the data set, which is what gets returned
def getNoisyData(csv_file='/home/julianalverio/catkin_ws/src/baxter_simulator/baxter_sim_examples/scripts/noisydata.csv'):
  reader = csv.reader(open(csv_file, 'r'))
  trajectories = list()
  for row in reader:
    trajectory = []
    for point in row:
      try:
        trajectory.append(eval(point))
      except SyntaxError:
        trajectory.append(eval('[' + re.sub(r'\s+', ',', row[0])[2:]))
    trajectories.append(trajectory)
  return trajectories


#used for testing purposes
#generate a simple trajectory to follow
def generateTestTrajectoryAngles(self):
  trajectory = list()
  starting_angles = {'left_w0': 0.6699952259595108,
                     'left_w1': 1.030009435085784,
                     'left_w2': -0.4999997247485215,
                     'left_e0': -1.189968899785275,
                     'left_e1': 1.9400238130755056,
                     'left_s0': -0.08000397926829805,
                     'left_s1': -0.9999781166910306}
  angles = [item[1] for item in starting_angles.items()]
  trajectory.append(angles, 10.)
  trajectory.append([1.5*angle for angle in angles], 10.)
  trajectory.append([2*angle for angle in angles], 10.)
  trajectory.append([1.2*angle for angle in angles], 10.)


#deprecated
def spawnURDF(xml):
  rospy.wait_for_service('/gazebo/spawn_urdf_model')
  try:
      spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
      resp_urdf = spawn_urdf("block", block_xml, "/",
                             block_pose, block_reference_frame)
  except rospy.ServiceException, e:
      rospy.logerr("Spawn URDF service call failed: {0}".format(e))


#deprecated
def testFollowTrajectoryFromJointVelocity(limb='left'):
  vels = [([0.1,0.1,0.5,0.5,0.5,0.5, 0.5], 1.), ([-0.1,-0.1,-0.5,-0.5,-0.5,-0.5, -0.5], 1.)]
  rospy.init_node("joint_velocity_client_%s" % limb)
  rospy.wait_for_message("/robot/sim/started", Empty)
  robo_controller = RobotController('left')
  robo_controller.followTrajectoryFromJointVelocity(vels)


#deprecated
def testFollowTrajectoryWithIK():
  limb='left'
  print("Initializing node... ")
  rospy.init_node("rsdk_joint_trajectory_client_%s" % (limb,))
  rospy.wait_for_message("/robot/sim/started", Empty)
  print("Getting robot state... ")
  rs = baxter_interface.RobotEnable(CHECK_VERSION)
  print("Enabling robot... ")
  rs.enable()
  print("Robot Enabled.")
  robo_controller = RobotController('left')
  trajectory = list()
  angles = [-0.11, -0.62, -1.15, 1.32,  0.80, 1.27,  2.39]
  trajectory.append((angles, 5.0))
  angles = [angle*0.5 for angle in angles]
  trajectory.append((angles, 5.0))
  angles = [angle*1.75 for angle in angles]
  trajectory.append((angles, 5.0))
  robo_controller.followTrajectoryWithIK(trajectory)


def testFollowTrajectoryFromJointAngles():
  rospy.init_node("joint_angle_follow")
  rospy.wait_for_message("/robot/sim/started", Empty)

  robo_controller = RobotController('left')
  starting_joint_angles = {'left_w0': 0.6699952259595108,
                       'left_w1': 1.030009435085784,
                       'left_w2': -0.4999997247485215,
                       'left_e0': -1.189968899785275,
                       'left_e1': 1.9400238130755056,
                       'left_s0': -0.08000397926829805,
                       'left_s1': -0.9999781166910306}
  robo_controller._limb.move_to_joint_positions(starting_joint_angles)

  #this trajectory was generated with the IK solver
  trajectory = [{'left_w0': 0.6663439517797247, 'left_w1': 1.0254832967958967, 'left_w2': -0.49791594416740903, 'left_e0': -1.1847989726061723, 'left_e1': 1.933762430285121, 'left_s0': -0.07890395814907691, 'left_s1': -0.9913289297553849}, {'left_w0': 0.7353491779669039, 'left_w1': 1.3090406232622902, 'left_w2': -0.21668752438391392, 'left_e0': -0.9407004398680272, 'left_e1': 1.2076700198856085, 'left_s0': -0.390958749829128, 'left_s1': -0.6675685155625416}, {'left_w0': 0.7330713167542067, 'left_w1': 1.3162117949266057, 'left_w2': 0.07355005285647863, 'left_e0': -0.929507737190373, 'left_e1': 1.166775364272026, 'left_s0': -0.12895727189225206, 'left_s1': -0.6402246273083545}, {'left_w0': 0.6662213273580528, 'left_w1': 1.7333649961627016, 'left_w2': 0.19624972970590862, 'left_e0': -0.9873696310196056, 'left_e1': 0.8023198330522042, 'left_s0': -0.20288995772020885, 'left_s1': -0.7783856313894464}, {'left_w0': 0.699089074999042, 'left_w1': 1.7495692436937351, 'left_w2': 0.5839992707054027, 'left_e0': -1.0308852381232938, 'left_e1': 0.7698219339384003, 'left_s0': -0.19511293413759315, 'left_s1': -0.7510437605072855}, {'left_w0': 0.6971546624084283, 'left_w1': 1.7674929011830742, 'left_w2': 0.4910183488567289, 'left_e0': -1.0107976354711212, 'left_e1': 0.7418040910707417, 'left_s0': -0.216089350524972, 'left_s1': -0.7452487873887369}, {'left_w0': 0.8543116593525074, 'left_w1': 1.595139000726061, 'left_w2': 0.27291051124465476, 'left_e0': -0.9543491138939362, 'left_e1': 1.0820713288402664, 'left_s0': -0.27074366544625333, 'left_s1': -0.9141157077556324}, {'left_w0': 0.7541181630688458, 'left_w1': 1.0843403328865067, 'left_w2': 0.13452227613101395, 'left_e0': -1.147813327587313, 'left_e1': 1.560391127177694, 'left_s0': 0.09741270782365391, 'left_s1': -1.1596446589326543}]
  robo_controller.followTrajectoryFromJointAngles(trajectory)
  second_trajectory = []
  for dictionary in trajectory:
    joint_angles = []
    joint_angles.append(dictionary['left_s0'])
    joint_angles.append(dictionary['left_s1'])
    joint_angles.append(dictionary['left_e0'])
    joint_angles.append(dictionary['left_e1'])
    joint_angles.append(dictionary['left_w0'])
    joint_angles.append(dictionary['left_w1'])
    joint_angles.append(dictionary['left_w2'])
    second_trajectory.append(joint_angles)
  robo_controller._limb.move_to_joint_positions(starting_joint_angles)
  robo_controller.followTrajectoryFromJointAngles(second_trajectory)
