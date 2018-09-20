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


# import physics_tests
# import scene_controller
# import trajectory
# import robot_controller
# import utils

from physics_tests import *
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

#stuff for callback functions

#for contact sensors
# def contactCallback(data):
#   if data.states:
#     print 'Contact detected!'
#     print data.states
    
#   rospy.Subscriber("/l_side_r_finger_contact_sensor_state", ContactsState, contactCallback)
#   rospy.Subscriber("/l_side_l_finger_contact_sensor_state", ContactsState, contactCallback)
#   rospy.Subscriber("/r_side_r_finger_contact_sensor_state", ContactsState, contactCallback)
#   rospy.Subscriber("/r_side_l_finger_contact_sensor_state", ContactsState, contactCallback)



  # deprecated version of the work table
  # models.append(Model(name='table', shape='block', roll=0., pitch=0., yaw=0., restitution_coeff=0., size_x=.7, size_y=1.5, size_z=.7, x=.4, y=0.1, z=0., mass=5000, color='Grey', mu1=1, mu2=1, reference_frame=''))

  # starting_angles = {'left_w0': 0.6699952259595108,
  #                  'left_w1': 1.030009435085784,
  #                  'left_w2': -0.4999997247485215,
  #                  'left_e0': -1.189968899785275,
  #                  'left_e1': 1.9400238130755056,
  #                  'left_s0': -0.08000397926829805,
  #                  'left_s1': -0.9999781166910306}
  manager = Manager()
  manager.robot_controller.jointAngleToPosition()
  assert False


  rospy.on_shutdown(manager.shutdown())
  trajectory = manager.robot_controller.generateTrajectoryPose()
  manager.robot_controller.followTrajectoryWithIK(trajectory)
  import pdb; pdb.set_trace()

  models = []
  import pdb; pdb.set_trace()

  models.append(Model(shape='box', size_x=0.1, size_y=0.2, size_z=0.3, x=0.1, y=0.2, z=0.3, mass=0.1, color='Red', mu1=1000, mu2=2000, restitution_coeff=0.5, roll=0.1, pitch=0.2, yaw=0.3, name="custom_name"))
  spawnGazeboModels(models)
  import pdb; pdb.set_trace()
  # zero_angles = {'left_w0':0,
  #                  'left_w1': 0,
  #                  'left_w2': 0,
  #                  'left_e0': 0,
  #                  'left_e1': 0,
  #                  'left_s0': 0,
  #                  'left_s1': 0}

  # small_angles = {'left_w0':0.1,
  #                  'left_w1': 0.1,
  #                  'left_w2': 0.1,
  #                  'left_e0': 0.1,
  #                  'left_e1': 0.1,
  #                  'left_s0': 0.1,
  #                  'left_s1': 0.1}
  # robot_controller._limb.move_to_joint_positions(zero_angles)
  # # robot_controller._limb.move_to_joint_positions(small_angles)
  # while not rospy.is_shutdown():
  #   pass
  # import sys; sys.exit()

  # rospy.init_node("ik_pick_and_place_demo")
  # rospy.wait_for_message("/robot/sim/started", Empty)
  # robot_controller = RobotController()
  # collectData()
  # getNoisyData()
  # import sys; sys.exit()
  # path = [[0.669995, 1.03001, -0.5, -1.18997, 1.94002, -0.080004, -0.999978], [0.668592, 1.02929, -0.0625, 3.05925e-316, 0.242503, 1.63e-322, 3.05914e-316]]
  # path = [[0.669995, 1.03001, -0.5, -1.18997, 1.94002, -0.080004, -0.999978], [0.668592, 1.02929, -0.0625, 3.05925e-316, 0.242503, 1.63e-322, 3.05914e-316]]
  # path = [[0.6699952259595108, 1.030009435085784, -0.4999997247485215, -1.189968899785275, 1.9400238130755056, -0.08000397926829805, -0.9999781166910306], [0.7781422377196167, 1.1356453883851858, -0.7216948208564031, -1.1197458028356198, 1.8300271990758368, -0.10690255084407813, -1.0155499499226386], [0.8564883103251449, 0.9911067717896954, -0.6952401377426854, -0.878188772394304, 1.8239147714614081, -0.10674760472604378, -1.0778672527047397], [0.8166595693096407, 0.9548243135177661, -0.7611120178415324, -0.6055581275382547, 1.859670853879711, -0.05374297138146601, -1.0119681410067372], [0.8302451720107575, 0.8109656199243157, -0.9044706895452012, -0.5624898986706034, 1.890948522455702, -0.1542134048983852, -0.8231800957714278], [0.5924553012481535, 0.8636312938214861, -0.9869668686412543, -0.43460788573460807, 1.9270345290196982, -0.21924839850714845, -0.8678021205374803], [0.569129628663293, 0.8177931206290384, -0.8170630762659061, -0.2842685276085715, 1.8421855110198067, -0.33021150086278833, -0.7398435816600969], [0.3588727520725308, 0.747245047192612, -0.7571469414984628, -0.14362391283924675, 1.8458740992496965, -0.2338470450962962, -0.6495969734505831], [0.28243492014001775, 0.8818088803162638, -0.7294828392766082, -0.058767417344444184, 1.6367523936121842, -0.34756262067076205, -0.6872902803916275], [0.2247562985275804, 1.010516275968612, -0.8746340993605678, 0.03180058541521873, 1.7140688138502316, -0.4814443436464452, -0.5571596271999999], [0.12164787222022597, 1.2400022793724512, -0.7788249494993481, -0.08133023441564557, 1.774388412481343, -0.4895425036944525, -0.5891411358923572], [0.09398519235450452, 1.4359460247215359, -0.7987525553545698, -0.2498672508383534, 1.6535743538549772, -0.45689303057987796, -0.5092852371434464]]
  # path = [[0.6699952259595108, 1.030009435085784, -0.4999997247485215, -1.189968899785275, 1.9400238130755056, -0.08000397926829805, -0.9999781166910306], [0.7822914236894253, 1.1204931483887393, -0.5937378962899262, -0.983665515461311, 1.9445961902869484, -0.018486387276214662, -0.8814495983631729], [0.9063739867874367, 1.2941943568367917, -0.5289969832624342, -0.8850346825315817, 1.8080443264320063, 0.0717023979705349, -0.8203631662515672], [0.8491973633563887, 1.371083873752349, -0.5628112089834415, -0.7250517919942816, 1.73778073445159, -0.12494125936498587, -0.7180147003741771], [0.7614142411036268, 1.467268452765302, -0.40403485986223375, -0.6283638518538183, 1.658528164065168, -0.13318960028318957, -0.5387518811448833], [0.7492080838878029, 1.3518868444001586, -0.48213395499929435, -0.6207513345828852, 1.4831939590822916, -0.3176348292791785, -0.4637948990797861], [0.7370019266719791, 1.2365052360350153, -0.5602330501363549, -0.6131388173119523, 1.3078597540994152, -0.5020800582751674, -0.38883791701468895], [0.7247957694561553, 1.121123627669872, -0.6383321452734155, -0.6055263000410193, 1.132525549116539, -0.6865252872711564, -0.31388093494959174], [0.713473354262754, 1.014095801262804, -0.7107767620135959, -0.598464939148772, 0.9698857738523418, -0.8576164362879198, -0.2443509381144592]]
  

  path = [[0.6699952259595108, 1.030009435085784, -0.4999997247485215, -1.189968899785275, 1.9400238130755056, -0.08000397926829805, -0.9999781166910306]]
  # path = [[0.6699952259595108, 1.030009435085784, -0.4999997247485215, -1.189968899785275, 1.9400238130755056, -0.08000397926829805, -0.9999781166910306], [0.585082959449846, 1.149134577732208, -0.4600305133317333, -1.012024528205707, 1.9899396181687823, -0.1684831846853534, -0.8418098115278596], [0.5476343433898576, 0.9948326773886624, -0.5458487255802914, -0.9229324019201325, 1.8465151314374963, -0.10124248847223336, -0.6856225599207553], [0.5073148877955518, 1.0567113835910167, -0.5285530074086975, -0.753682383718749, 1.644443155944087, -0.16993071305479282, -0.585375359386533], [0.4195183399100322, 1.1818125406461275, -0.712165745817032, -0.9010573138917103, 1.5769849626783419, -0.2366538254575999, -0.5384085906434184], [0.25282196788166167, 1.2246487809296362, -0.7358004429722331, -0.9289204514549982, 1.3864661210256444, -0.3527816791488043, -0.6346339266981831], [0.3948260722272491, 1.1597420245866787, -0.728086441653628, -0.8270515752539258, 1.2580477099728675, -0.5084061050150818, -0.5143221510829005], [0.5368301765728365, 1.094835268243721, -0.7203724403350229, -0.7251826990528534, 1.1296292989200905, -0.6640305308813592, -0.39401037546761786], [0.6788342809184239, 1.0299285119007635, -0.7126584390164178, -0.6233138228517808, 1.0012108878673136, -0.8196549567476367, -0.2736985998523353], [0.713473354262754, 1.014095801262804, -0.7107767620135959, -0.598464939148772, 0.9698857738523418, -0.8576164362879198, -0.2443509381144592]]
  for angles in path:
      joints = ['left_w0', 'left_w1', 'left_w2', 'left_e0', 'left_e1', 'left_s0', 'left_s1']
      move_angles = dict(zip(joints, angles))
      robot_controller._limb.move_to_joint_positions(move_angles)
  print 'NOW USING STANDARD PATH'
  start_dict = {'left_w0': 0.6699952259595108, 'left_w1': 1.030009435085784, 'left_w2': -0.4999997247485215,'left_e0': -1.189968899785275,'left_e1': 1.9400238130755056,'left_s0': -0.08000397926829805,'left_s1': -0.9999781166910306}
  goal_dict = {'left_w0': 0.713473354262754, 'left_w1': 1.014095801262804, 'left_w2': -0.7107767620135959, 'left_e0': -0.598464939148772, 'left_e1': 0.9698857738523418, 'left_s0': -0.8576164362879198, 'left_s1': -0.2443509381144592}
  robot_controller._limb.move_to_joint_positions(start_dict)

  robot_controller._limb.move_to_joint_positions(goal_dict)
  return



  pre_grip_angles = {'left_w0': 0.713473354262754, 'left_w1': 1.014095801262804, 'left_w2': -0.7107767620135959, 'left_e0': -0.598464939148772, 'left_e1': 0.9698857738523418, 'left_s0': -0.8576164362879198, 'left_s1': -0.2443509381144592}
  generateNoisyData(ending_angles_dict=pre_grip_angles)
  trajectories = getNoisyData()
  # robot_controller = RobotController()
  # final_positions = [pair[1] for pair in trajectories]
  # for index, angles in enumerate(final_positions):
  #   print 'moving to', index
  #   robot_controller._limb.move_to_joint_positions(eval(angles))
  return

  # robot_controller = RobotController()
  # robot_controller._limb.move_to_joint_positions(starting_angles)

  current_pose = robot_controller._limb.endpoint_pose()
  next_pose = Pose()
  next_pose.position.x = current_pose['position'].x + 0.1
  next_pose.position.y = current_pose['position'].y - 0.1
  next_pose.position.z = current_pose['position'].z + 0.1
  next_pose.orientation.x = current_pose['orientation'].x
  next_pose.orientation.y = current_pose['orientation'].y - 0.1
  next_pose.orientation.z = current_pose['orientation'].z - 0.1
  next_pose.orientation.w = current_pose['orientation'].w + 0.1
  next_angles = robot_controller.solveIK(next_pose)
  print "Showing Next Angles:"
  print next_angles
  robot_controller._limb.move_to_joint_positions(next_angles)

  return
  

  return edgeCases()
  # return balldrop()


  rospy.init_node("ik_pick_and_place_demo")
  rospy.wait_for_message("/robot/sim/started", Empty)


  rospy.wait_for_service('/gazebo/spawn_sdf_model')
  spawn_proxy = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
  pose = Pose(position=Point(x=0.8, y=0., z=0.6))
  reference_frame = 'world'
  sdf_file = open('/home/julianalverio/catkin_ws/src/baxter_simulator/baxter_sim_examples/scripts/ball.sdf', 'r')
  sdf = sdf_file.read().replace('\n', '')
  sdf_file.close()
  resp_sdf = spawn_proxy('bouncy_ball', sdf, "/",
                         pose, reference_frame)

  delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

  def delete():
    delete_model('bouncy_ball')

  rospy.on_shutdown(delete)

  print "ALL DONE"
  while not rospy.is_shutdown():
    pass

  return


if __name__ == '__main__':
  sys.exit(main())