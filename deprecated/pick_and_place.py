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






COLORS = set(['Grey', 'DarkGrey', 'FlatBlack', 'Black', 'Red', 'RedBright', 'Green', 'Blue', 
'SkyBlue', 'Yellow', 'ZincYellow', 'DarkYellow', 'Purple', 'Turquoise', 'Orange',
'Indigo', 'White', 'RedGlow', 'Green', 'Blue', 'RedTransparentOverlay', 'BlueTransparentOverlay',
'GreenTransparentOverlay', 'OrangeTransparentOverlay', 'DarkOrangeTransparentOverlay',
'RedTransparent', 'BlueTransparent', 'GreenTransparent', 'DarkMagentaTransparent',
'GreyTransparent', 'BlackTransparent', 'YellowTransparent'])



class PickAndPlace(object):
  def __init__(self, limb='left', hover_distance = 0.15, verbose=True):
    self._limb_name = limb # string
    self._hover_distance = hover_distance # in meters
    self._verbose = verbose # bool
    self._limb = baxter_interface.Limb(limb)
    self._gripper = baxter_interface.Gripper(limb)
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    rospy.wait_for_service(ns, 5.0)
    # verify robot is enabled
    print("Getting robot state... ")
    self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
    self._init_state = self._rs.state().enabled
    print("Enabling robot... ")
    self._rs.enable()

  def gripper_open(self):
    self._gripper.open()
    rospy.sleep(1.0)

  def gripper_close(self):
    self._gripper.close()
    rospy.sleep(1.0)

  def move_to_start(self, start_angles=None):
    print("Moving the %s arm to start pose..." % (self._limb_name))
    if not start_angles:
        start_angles = dict(zip(self._joint_names, [0]*7))
    self._limb.move_to_joint_positions(start_angles)
    self.gripper_open()
    rospy.sleep(1.0)
    print("Running. Ctrl-c to quit")


  def solveIK(self, pose, timestamp=None, frame_id='base', verbose=True):
    if timestamp == None:
      timestamp = rospy.Time.now()
    header = Header(stamp=timestamp, frame_id=frame_id)
    ikreq = SolvePositionIKRequest()
    ikreq.pose_stamp.append(PoseStamped(header=header, pose=pose))
    try:
      resp = self._iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
      rospy.logerr("Service call failed: %s" % (e,))
      return

    resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
    if (resp_seeds[0] != resp.RESULT_INVALID):
      seed_str = {
                  ikreq.SEED_USER: 'User Provided Seed',
                  ikreq.SEED_CURRENT: 'Current Joint Angles',
                  ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                 }.get(resp_seeds[0], 'None')
      # Format solution into Limb API-compatible dictionary
      joint_angles = dict(zip(resp.joints[0].name, resp.joints[0].position))
      if verbose:
        print("IK Joint Solution:\n{0}".format(joint_angles))
        print("------------------")
      return joint_angles
    else:
      rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
      

  def generateTrajectoryAngles(self):

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






  #translate in x, y, z by 0.2 meteres then add pi/4 rotation to x, y, z, w
  #returns list of [pose, timestamp] pairs
  def generateTrajectoryPose(self, timestamped=True):
    poses = []
    now = rospy.Time.now()
    SECOND = rospy.Duration(1)
    current_pose = self._limb.endpoint_pose()

    next_pose = Pose()
    next_pose.position.x = current_pose['position'].x
    next_pose.position.y = current_pose['position'].y
    next_pose.position.z = current_pose['position'].z
    next_pose.orientation.x = current_pose['orientation'].x
    next_pose.orientation.y = current_pose['orientation'].y
    next_pose.orientation.z = current_pose['orientation'].z
    next_pose.orientation.w = current_pose['orientation'].w
    poses.append([next_pose, now])
    current_pose = next_pose

    next_pose = copy.deepcopy(current_pose)
    next_pose.position.x += 0.2
    poses.append([next_pose, now + SECOND])
    current_pose = next_pose

    next_pose = copy.deepcopy(current_pose)
    next_pose.position.y += 0.2
    poses.append([next_pose, now + 2*SECOND])
    current_pose = copy.deepcopy(next_pose)

    next_pose = copy.deepcopy(current_pose)
    next_pose.position.z += 0.2
    poses.append([next_pose, now + 3*SECOND])
    current_pose = copy.deepcopy(next_pose)

    next_pose = copy.deepcopy(current_pose)
    next_pose.orientation.x += 0.2
    poses.append([next_pose, now + 4*SECOND])
    current_pose = copy.deepcopy(next_pose)

    next_pose = copy.deepcopy(current_pose)
    next_pose.orientation.y += 0.2
    poses.append([next_pose, now + 5*SECOND])
    current_pose = copy.deepcopy(next_pose)

    next_pose = copy.deepcopy(current_pose)
    next_pose.orientation.z += 0.2
    poses.append([next_pose, now + 6*SECOND])
    current_pose = copy.deepcopy(next_pose)

    next_pose = copy.deepcopy(current_pose)
    next_pose.orientation.w += 0.2
    poses.append([next_pose, now + 7*SECOND])
    current_pose = copy.deepcopy(next_pose)

    if timestamped: return poses
    return [pair[0] for pair in poses]

  #input: list of (angle, timestamp) or (angle, timedelta as float) pairs
  #angle can be dict or list in order: 's0, s1, e0, e1, w0, w1, w2' (limb_interface.joint_names())
  def followTrajectoryWithIK(self, trajectory, time_buffer=2., wait=True):
    limb = self._limb.name
    limb_interface = self._limb
    joints = limb_interface.joint_names()

    #process joint angles
    if type(trajectory[0][0]) == dict:
      for idx, (angle, timestamp) in enumerate(trajectory):
        trajectory[idx] = ([limb_interface.joint_angle(joint) for joint in joints], timestamp)
    elif type(trajectory[0][0]) != list:
      raise InvalidInputException()

    if type(trajectory[0][1]) in [rospy.Duration, genpy.rostime.Duration]:
      for idx, (angles, timestamp) in enumerate(trajectory):
        trajectory[idx] = (angles, timestamp.to_sec())
    elif type(trajectory[0][1]) not in [float, int]:
      raise InvalidInputException()
    

    traj = Trajectory(limb)
    rospy.on_shutdown(traj.stop)
    # Command Current Joint Positions first
    current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
    traj.add_point(current_angles, 0.0)


    for angles, delta in trajectory:
      traj.add_point(angles, delta)


    execution_time = sum([pair[1] for pair in trajectory])

    traj.start()
    if wait:
      traj.wait(execution_time + time_buffer)

    print("TRAJECTORY COMPLETED")



  #linearly interpolates between joint angles
  #input can be a list of length-7 list with angles in order: 's0, s1, e0, e1, w0, w1, w2' (limb_interface.joint_names())
  #input can be list of dictionaries with joints as keys and angles as values
  def followTrajectoryFromJointAngles(self, input_trajectory):
    if type(input_trajectory[0]) == list:
      prefix = self._limb.name + '_'
      trajectory = list()
      for input_list in input_trajectory:
        trajectory_dict = dict()
        trajectory_dict[prefix+'s0'] = input_list[0]
        trajectory_dict[prefix+'s1'] = input_list[1]
        trajectory_dict[prefix+'e0'] = input_list[2]
        trajectory_dict[prefix+'e1'] = input_list[3]
        trajectory_dict[prefix+'w0'] = input_list[4]
        trajectory_dict[prefix+'w1'] = input_list[5]
        trajectory_dict[prefix+'w2'] = input_list[6]
        trajectory.append(trajectory_dict)

    elif type(input_trajectory[0]) == dict:
      trajectory = input_trajectory

    else:
      raise InvalidInputException()

    for joint_angles in trajectory:
      self._limb.move_to_joint_positions(joint_angles)


  #input: list of (velocities, timedelta) pairs
  #velocities must be lists in order: 's0, s1, e0, e1, w0, w1, w2' (limb_interface.joint_names())
  #timedeltas must be floats
  def followTrajectoryFromJointVelocity(self, trajectory, stop=True):
    topic = 'robot/limb/%s/joint_command' % self._limb.name
    pub = rospy.Publisher(topic, JointCommand, queue_size=10)
    rate = rospy.Rate(50.)

    VELOCITY_MODE = 2
    template = JointCommand()
    template.mode = VELOCITY_MODE
    template.names.extend(self._limb.joint_names())
    for pair in trajectory:
      velocities, delta = pair
      msg = copy.deepcopy(template)
      msg.command.extend(velocities)
      pub.publish(msg)
      rospy.sleep(delta)

    if stop:
      velocities = [0.0] * len(self._limb.joint_names())
      cmd = dict(zip(self._limb.joint_names(), velocities))
      self._limb.set_joint_velocities(cmd)



class Trajectory(object):
  def __init__(self, limb):
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




class InvalidInputException(Exception):
  pass





# def start_server(limb='left', rate=100., mode='position', interpolation='bezier'):
#     print("Initializing node... ")
#     rospy.init_node("rsdk_%s_joint_trajectory_action_server%s" %
#                     (mode, "" if limb == 'both' else "_" + limb,))
#     print("Initializing joint trajectory action server...")

#     if mode == 'velocity':
#         dyn_cfg_srv = Server(VelocityJointTrajectoryActionServerConfig,
#                              lambda config, level: config)
#     elif mode == 'position':
#         dyn_cfg_srv = Server(PositionJointTrajectoryActionServerConfig,
#                              lambda config, level: config)
#     else:
#         dyn_cfg_srv = Server(PositionFFJointTrajectoryActionServerConfig,
#                              lambda config, level: config)
#     jtas = []
#     if limb == 'both':
#         jtas.append(JointTrajectoryActionServer('right', dyn_cfg_srv,
#                                                 rate, mode, interpolation))
#         jtas.append(JointTrajectoryActionServer('left', dyn_cfg_srv,
#                                                 rate, mode, interpolation))
#     else:
#         jtas.append(JointTrajectoryActionServer(limb, dyn_cfg_srv, rate, mode, interpolation))

#     def cleanup():
#         for j in jtas:
#             j.clean_shutdown()

#     rospy.on_shutdown(cleanup)
#     print("Running. Ctrl-c to quit")
#     rospy.spin()


#Spawn SDF models from string with no file I/O
#input: a list of Model objects
def loadGazeboModelsNoIO(models):
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_proxy = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    index = 0
    for model in models:
      pose = model.pose
      reference_frame = model.reference_frame
      sdf = generateSDF(model, index).replace('\n', '')
      try:
        print 'loading model named: %s' % model.name
        resp_sdf = spawn_proxy(model.name, sdf, "/",
                             pose, reference_frame)
        index += 1
      except rospy.ServiceException, e:
          rospy.logerr("Spawn SDF service call failed: {0}".format(e))



'''
#Takes in a list of tuples. Tuples must be of one of the following types:
#(sdf_path, model_name), (None, model_object'''
def load_gazebo_models(models,
                       table_pose=Pose(position=Point(x=1.0, y=0.0, z=0.0)),
                       table_reference_frame="world"):

  model_dir = rospkg.RosPack().get_path('baxter_sim_examples')+"/models"
  def spawnSDF(sdf_path, model):
    if not sdf_path:
      sdf_path = '%s/%s/%s.sdf' % (model_dir, model.name, model.name)
    sdf_file = open(sdf_path, 'r')
    xml = sdf_file.read().replace('\n', '')
    sdf_file.close()

    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        if type(model) == str:
          pose = Pose(position=Point(x=1.0, y=0.0, z=0.0))
          reference_frame = 'world'
        else:
          pose = model.pose
          reference_frame = model.reference_frame
        resp_sdf = spawn_sdf(str(model), xml, "/",
                               pose, reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))



  for (sdf_path, model) in models:
    #get path to sdf file as .../models/object1/object1.sdf
    spawnSDF(sdf_path, model)


  # ##CURRENTLY UNUSED
  # def spawnURDF(xml):
  #   rospy.wait_for_service('/gazebo/spawn_urdf_model')
  #   try:
  #       spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
  #       resp_urdf = spawn_urdf("block", block_xml, "/",
  #                              block_pose, block_reference_frame)
  #   except rospy.ServiceException, e:
  #       rospy.logerr("Spawn URDF service call failed: {0}".format(e))



def delete_gazebo_models():
  # This will be called on ROS Exit, deleting Gazebo models
  # Do not wait for the Gazebo Delete Model service, since
  # Gazebo should already be running. If the service is not
  # available since Gazebo has been killed, it is fine to error out
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    except rospy.ServiceException, _:
        print 'FAILED TO CONTACT SERVICE PROXY: /gazebo/delete_model'

    model_dir = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
    models = os.listdir(model_dir)
    for model in models:
      try:
        resp_delete = delete_model(model)
      except rospy.ServiceException, _:
        print 'FAILED TO DELETE MODEL: %s' % model


class Model(object):
  def __init__(self, shape='block', size_x=0.5, size_y=0.5, 
               size_z=0.5, size_r=0.5, x=None, y=None, z=None, 
               mass=0.5, color=None, mu1=1000, mu2=1000,
               reference_frame='world',
               restitution_coeff=0.5, roll=0., pitch=0., yaw=0.,
               name=None):
    self.shape = shape
    if self.shape not in ['block', 'cylinder', 'sphere']:
      self.shape = 'block'
    self.size_x = size_x
    self.size_y = size_y
    self.size_z = size_z
    self.size_r = size_r

    self.x = x
    if not x:
      self.x = random.uniform(-2, 2)
    self.y = y
    if y == None:
      self.y = random.uniform(-2, 2)
    self.z = z
    if z == None:
      print 'I found size_z == None'
      self.z = -size_z/2.0
    self.mass = mass
    self.color = color
    if self.color not in COLORS:
      self.color = random.sample(COLORS, 1)
    self.mu1 = mu1
    self.mu2 = mu2

    moments = self.getMoments()
    self.ixx = moments[0]
    self.ixy = 0.
    self.ixz = 0.
    self.iyy = moments[1]
    self.iyz = 0.
    self.izz = moments[2]
    self.name = name

    self.pose = Pose(position=Point(x=self.x, y=self.y, z=self.z))
    self.reference_frame = reference_frame
    self.COR = restitution_coeff

    self.roll = roll
    self.pitch = pitch
    self.yaw = yaw



  def __str__(self):
    return self.name


  #assumption: x_size is width, y_size is depth, and z_size is height
  #reference: dynref.engr.illinois.edu/rem.html
  #returns: [ixx, iyy, izz]
  def getMoments(self):
    moments = np.ones((1, 3))
    if self.shape == 'block':
      moments *= 1./12 * self.mass
      moments[0][0] *= self.size_y**2 + self.size_z**2
      moments[0][1] *= self.size_x**2 + self.size_z**2
      moments[0][2] *= self.size_x**2 + self.size_y**2

    elif self.shape == 'sphere':
      moments *=  0.4 * self.mass * self.size_r**2

    elif self.shape == 'cylinder':
      moments[0][0] = 1./12 * self.mass * self.size_z**2
      moments [0][0] += 0.25 * self.mass * self.size_r**2
      moments[0][1] = moments[0][0]
      moments[0][2] = 0.5 * self.mass * self.size_r**2

    return moments.tolist()[0]



#Takes in a Model object
#Returns SDF string
def generateSDF(model, index=0):
  sdf = ''
  if not model.name:
    name = 'object%d' % index
    model.name = name
  else:
    name = model.name
  sdf += '<robot name="%s">\n' % name
  sdf += '\t<link name="%s">\n' % name
  sdf += '\t\t<inertial>\n'
  # sdf += '\t\t\t<origin xyz="%f %f %f" rpy="%f %f %f" />\n' % (0,0,0,0,0,0)
  sdf += '\t\t\t<origin xyz="%f %f %f" rpy="%f %f %f" />\n' % (model.x, model.y, model.z, model.roll, model.pitch, model.yaw)
  sdf += '\t\t\t<mass value="%s" />\n' % model.mass
  sdf += '\t\t\t<inertia  ixx="%f" ixy="%f"  ixz="%f"  iyy="%f"  iyz="%f"  izz="%f" />\n' % (model.ixx, model.ixy, model.ixz, model.iyy, model.iyz, model.izz)
  sdf += '\t\t</inertial>\n'
  sdf += '\t\t<visual>\n'
  sdf += '\t\t\t<origin xyz="%f %f %f" rpy="%f %f %f"/>\n' % (model.x, model.y, model.z, model.roll, model.pitch, model.yaw)
  # sdf += '\t\t\t<origin xyz="%f %f %f" rpy="%f %f %f"/>\n' % (0,0,0,0,0,0)
  sdf += '\t\t\t<geometry>\n'
  if model.shape == 'block':
    sdf += '\t\t\t\t<box size="%f %f %f" />\n' % (model.size_x, model.size_y, model.size_z)
  elif model.shape == 'cylinder':
    sdf += '\t\t\t\t<cylinder radius="%f" length="%f" />\n' % (model.size_r, model.size_z)
  else:
    sdf += '\t\t\t\t<sphere radius="%f" />\n' % (model.size_r)
  sdf += '\t\t\t</geometry>\n'
  sdf += '\t\t</visual>\n'
  sdf += '\t\t<collision>\n'
  # sdf += '\t\t\t<origin xyz="%f %f %f" rpy="%f %f %f"/>\n' % (0,0,0,0,0,0)
  sdf += '\t\t\t<origin xyz="%f %f %f" rpy="%f %f %f"/>\n' % (model.x, model.y, model.z, model.roll, model.pitch, model.yaw)
  sdf += '\t\t\t<geometry>\n'
  if model.shape == 'block':
    sdf += '\t\t\t\t<box size="%f %f %f" />\n' % (model.size_x, model.size_y, model.size_z)
  elif model.shape == 'cylinder':
    sdf += '\t\t\t\t<cylinder radius="%f" length="%f" />\n' % (model.size_r, model.size_z)
  else:
    sdf += '\t\t\t\t<sphere radius="%f" />\n' % (model.size_r)
  sdf += '\t\t\t</geometry>\n'
  sdf += '\t\t\t<surface>\n'
  sdf += '\t\t\t\t<bounce>\n'
  sdf += '\t\t\t\t\t<restitution_coefficient>%s</restitution_coefficient>\n' % model.COR
  sdf += '\t\t\t\t\t<threshold>0</threshold>\n'
  sdf += '\t\t\t\t</bounce>\n'
  sdf += '\t\t\t\t<contact>\n'
  # sdf += '\t\t\t\t\t<ode>\n'
  # sdf += '\t\t\t\t\t\t<max_vel>10000</max_vel>\n'
  # sdf += '\t\t\t\t\t</ode>\n'
  sdf += '\t\t\t\t</contact>\n'
  sdf += '\t\t\t</surface>\n'
  sdf += '\t\t</collision>\n'
  sdf += '\t</link>\n'
  sdf += '\t<gazebo reference="%s">\n' % name
  sdf += '\t\t<material>Gazebo/%s</material>\n' % model.color
  sdf += '\t\t\t<mu1>%f</mu1>\n' % model.mu1
  sdf += '\t\t\t<mu2>%f</mu1>\n' % model.mu2
  sdf += '\t</gazebo>\n'
  sdf += '</robot>\n'
  return sdf



#Takes in a list of tuples. Tuples must be of one of the following types:
#(sdf_path, model_name), (None, model_object)
def generateModels(input_models):
  #remove old objects
  # model_dir = '/home/julianalverio/catkin_ws/src/baxter_simulator/baxter_sim_examples/models/'
  model_dir = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
  object_regex = re.compile('object\d+')
  for file in os.listdir(model_dir):
    if object_regex.match(file):
      print 'deleting %s%s' % (model_dir, file)
      os.system('rm -rf %s%s' % (model_dir, file))

  for index, (sdf, model) in enumerate(input_models):
    if type(model) == str: continue
    name = 'object%d' % index
    model.name = name
    os.makedirs('%s%s' % (model_dir, name))
    sdf = open('%s%s/%s.sdf' % (model_dir, name,name), 'w')
    sdf.write('<robot name="%s">\n' % name)
    sdf.write('\t<link name="%s">\n' % name)
    sdf.write('\t\t<inertial>\n')
    sdf.write('\t\t\t<origin xyz="%f %f %f" rpy="%f %f %f" />\n' % (model.x, model.y, model.z, model.roll, model.pitch, model.yaw))
    sdf.write('\t\t\t<mass value="%s" />\n' % model.mass)
    sdf.write('\t\t\t<inertia  ixx="%f" ixy="%f"  ixz="%f"  iyy="%f"  iyz="%f"  izz="%f" />\n' % (model.ixx, model.ixy, model.ixz, model.iyy, model.iyz, model.izz))
    sdf.write('\t\t</inertial>\n')
    sdf.write('\t\t<visual>\n')
    sdf.write('\t\t\t<origin xyz="%f %f %f" rpy="%f %f %f"/>\n' % (model.x, model.y, model.z, model.roll, model.pitch, model.yaw))
    sdf.write('\t\t\t<geometry>\n')
    if model.shape == 'block':
      sdf.write('\t\t\t\t<box size="%f %f %f" />\n' % (model.size_x, model.size_y, model.size_z))
    elif model.shape == 'cylinder':
      sdf.write('\t\t\t\t<cylinder radius="%f" length="%f" />\n' % (model.size_r, model.size_z))
    else:
      sdf.write('\t\t\t\t<sphere radius="%f" />\n' % (model.size_r))
    sdf.write('\t\t\t</geometry>\n')
    sdf.write('\t\t</visual>\n')
    sdf.write('\t\t<collision>\n')
    sdf.write('\t\t\t<origin xyz="%f %f %f" rpy="%f %f %f"/>\n' % (model.x, model.y, model.z, model.roll, model.pitch, model.yaw))
    sdf.write('\t\t\t<geometry>\n')
    if model.shape == 'block':
      sdf.write('\t\t\t\t<box size="%f %f %f" />\n' % (model.size_x, model.size_y, model.size_z))
    elif model.shape == 'cylinder':
      sdf.write('\t\t\t\t<cylinder radius="%f" length="%f" />\n' % (model.size_r, model.size_z))
    else:
      sdf.write('\t\t\t\t<sphere radius="%f" />\n' % (model.size_r))
    sdf.write('\t\t\t</geometry>\n')
    sdf.write('\t\t\t<surface>\n')
    sdf.write('\t\t\t\t<bounce>\n')
    sdf.write('\t\t\t\t\t<restitution_coefficient>%s</restitution_coefficient>\n' % model.COR)
    sdf.write('\t\t\t\t\t<threshold>0</threshold>\n')
    sdf.write('\t\t\t\t</bounce>\n')
    sdf.write('\t\t\t\t<contact>\n')
    # sdf.write('\t\t\t\t\t<ode>\n')
    # sdf.write('\t\t\t\t\t\t<max_vel>10000</max_vel>\n')
    # sdf.write('\t\t\t\t\t</ode>\n')
    sdf.write('\t\t\t\t</contact>\n')
    sdf.write('\t\t\t</surface>\n')
    sdf.write('\t\t</collision>\n')
    sdf.write('\t</link>\n')
    sdf.write('\t<gazebo reference="%s">\n' % name)
    sdf.write('\t\t<material>Gazebo/%s</material>\n' % model.color)
    sdf.write('\t\t\t<mu1>%f</mu1>\n' % model.mu1)
    sdf.write('\t\t\t<mu2>%f</mu2>\n' % model.mu2)
    sdf.write('\t</gazebo>\n')
    sdf.write('</robot>\n')
    sdf.close()



def testFollowTrajectoryFromJointVelocity(limb='left'):
  vels = [([0.1,0.1,0.5,0.5,0.5,0.5, 0.5], 1.), ([-0.1,-0.1,-0.5,-0.5,-0.5,-0.5, -0.5], 1.)]
  rospy.init_node("joint_velocity_client_%s" % limb)
  rospy.wait_for_message("/robot/sim/started", Empty)
  pnp = PickAndPlace('left')
  pnp.followTrajectoryFromJointVelocity(vels)



def testFollowTrajectoryWithIK():
  # rospy.init_node("ik_pick_and_place_demo")
  # rospy.wait_for_message("/robot/sim/started", Empty)

  limb='left'

  print("Initializing node... ")
  rospy.init_node("rsdk_joint_trajectory_client_%s" % (limb,))
  # rospy.wait_for_message("/robot/sim/started", Empty)
  print("Getting robot state... ")
  rs = baxter_interface.RobotEnable(CHECK_VERSION)
  print("Enabling robot... ")
  rs.enable()
  print("Robot Enabled.")


  pnp = PickAndPlace('left')


    
  trajectory = list()
  angles = [-0.11, -0.62, -1.15, 1.32,  0.80, 1.27,  2.39]
  trajectory.append((angles, 5.0))
  angles = [angle*0.5 for angle in angles]
  trajectory.append((angles, 5.0))
  angles = [angle*1.75 for angle in angles]
  trajectory.append((angles, 5.0))

  pnp.followTrajectoryWithIK(trajectory)


def testFollowTrajectoryFromJointAngles():
  rospy.init_node("ik_pick_and_place_demo")
  rospy.wait_for_message("/robot/sim/started", Empty)


  pnp = PickAndPlace('left')
  starting_joint_angles = {'left_w0': 0.6699952259595108,
                       'left_w1': 1.030009435085784,
                       'left_w2': -0.4999997247485215,
                       'left_e0': -1.189968899785275,
                       'left_e1': 1.9400238130755056,
                       'left_s0': -0.08000397926829805,
                       'left_s1': -0.9999781166910306}
  pnp._limb.move_to_joint_positions(starting_joint_angles)

  #this trajectory was generated with the IK solver
  trajectory = [{'left_w0': 0.6663439517797247, 'left_w1': 1.0254832967958967, 'left_w2': -0.49791594416740903, 'left_e0': -1.1847989726061723, 'left_e1': 1.933762430285121, 'left_s0': -0.07890395814907691, 'left_s1': -0.9913289297553849}, {'left_w0': 0.7353491779669039, 'left_w1': 1.3090406232622902, 'left_w2': -0.21668752438391392, 'left_e0': -0.9407004398680272, 'left_e1': 1.2076700198856085, 'left_s0': -0.390958749829128, 'left_s1': -0.6675685155625416}, {'left_w0': 0.7330713167542067, 'left_w1': 1.3162117949266057, 'left_w2': 0.07355005285647863, 'left_e0': -0.929507737190373, 'left_e1': 1.166775364272026, 'left_s0': -0.12895727189225206, 'left_s1': -0.6402246273083545}, {'left_w0': 0.6662213273580528, 'left_w1': 1.7333649961627016, 'left_w2': 0.19624972970590862, 'left_e0': -0.9873696310196056, 'left_e1': 0.8023198330522042, 'left_s0': -0.20288995772020885, 'left_s1': -0.7783856313894464}, {'left_w0': 0.699089074999042, 'left_w1': 1.7495692436937351, 'left_w2': 0.5839992707054027, 'left_e0': -1.0308852381232938, 'left_e1': 0.7698219339384003, 'left_s0': -0.19511293413759315, 'left_s1': -0.7510437605072855}, {'left_w0': 0.6971546624084283, 'left_w1': 1.7674929011830742, 'left_w2': 0.4910183488567289, 'left_e0': -1.0107976354711212, 'left_e1': 0.7418040910707417, 'left_s0': -0.216089350524972, 'left_s1': -0.7452487873887369}, {'left_w0': 0.8543116593525074, 'left_w1': 1.595139000726061, 'left_w2': 0.27291051124465476, 'left_e0': -0.9543491138939362, 'left_e1': 1.0820713288402664, 'left_s0': -0.27074366544625333, 'left_s1': -0.9141157077556324}, {'left_w0': 0.7541181630688458, 'left_w1': 1.0843403328865067, 'left_w2': 0.13452227613101395, 'left_e0': -1.147813327587313, 'left_e1': 1.560391127177694, 'left_s0': 0.09741270782365391, 'left_s1': -1.1596446589326543}]
  pnp.followTrajectoryFromJointAngles(trajectory)
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
  pnp._limb.move_to_joint_positions(starting_joint_angles)



  pnp.followTrajectoryFromJointAngles(second_trajectory)



class simHandler(object):

  def __init__(self):
    self.srv = placeholder()
    self.pause_proxy = rospy.ServiceProxy('/gazebo/pause_physics', placeholder)
    self.unpause_proxy = rospy.ServiceProxy('/gazebo/unpause_physics', placeholder)

  def pause(self):
    self.pause_proxy()

  def unpause(self):
    self.unpause_proxy()


def externalCameraDemo():

  def externalCameraDemoBackup():
    sdf_path = '/home/julianalverio/catkin_ws/src/baxter_simulator/baxter_sim_examples/scripts/camera.sdf'
    sdf = open(sdf_path, 'r').read().replace('\n', '')
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_proxy = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    index = 0
    pose = Pose()
    pose.position.x = 2.5
    pose.position.y = 0.0
    pose.position.z = 1.5
    pose.orientation.x = -0.15
    pose.orientation.y = 0
    pose.orientation.z = .98
    pose.orientation.w = .0

    resp_sdf = spawn_proxy('external_camera', sdf, "/",
                           pose, 'world')

  rospy.init_node("ik_pick_and_place_demo")
  rospy.wait_for_message("/robot/sim/started", Empty)

  rospy.on_shutdown(cleanUp)

  ##work table
  models = list()
  models.append(Model(name='table', shape='block', roll=0., pitch=0., yaw=0., restitution_coeff=0., size_x=.7, size_y=1.5, size_z=.7, x=.4, y=0.1, z=0., mass=5000, color='Grey', mu1=1, mu2=1, reference_frame=''))
  
  print 'generating table'
  loadGazeboModelsNoIO(models)

  print 'generating camera'
  externalCameraDemoBackup()


  models = []
  models.append(Model(name='test1', shape='cylinder', size_r=0.025, size_z=0.1, x=0.4, y=0.1, z=0.5, mass=1, color='Green', mu1=1000, mu2=1000, reference_frame=''))
  models.append(Model(name='test2', shape='cylinder', size_r=0.025, size_z=0.1, x=0.4, y=-0.1, z=0.5, mass=1, color='Red', mu1=1000, mu2=1000, reference_frame=''))
  models.append(Model(name='test3', shape='cylinder', size_r=0.025, size_z=0.1, x=.45, y=0., z=0.5, mass=1, color='Blue', mu1=1000, mu2=1000, reference_frame=''))
  loadGazeboModelsNoIO(models)

  while not rospy.is_shutdown():
    pass
  return



def balldrop():

  def externalCameraDemoBackup():
    sdf_path = '/home/julianalverio/catkin_ws/src/baxter_simulator/baxter_sim_examples/scripts/camera.sdf'
    sdf = open(sdf_path, 'r').read().replace('\n', '')
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_proxy = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    index = 0
    pose = Pose()
    pose.position.x = 2.5
    pose.position.y = 0.0
    pose.position.z = 1.5
    pose.orientation.x = -0.15
    pose.orientation.y = 0
    pose.orientation.z = .98
    pose.orientation.w = .0

    resp_sdf = spawn_proxy('external_camera', sdf, "/",
                           pose, 'world')

  rospy.init_node("ik_pick_and_place_demo")
  rospy.wait_for_message("/robot/sim/started", Empty)

  rospy.on_shutdown(cleanUp)

  rospy.wait_for_service('/gazebo/spawn_sdf_model')
  spawn_proxy = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
  pose = Pose(position=Point(x=1, y=0., z=1))
  reference_frame = ''
  sdf_file = open('/home/julianalverio/catkin_ws/src/baxter_simulator/baxter_sim_examples/scripts/bouncy_ball.sdf', 'r')
  sdf = sdf_file.read().replace('\n', '')
  sdf_file.close()
  print sdf
  resp_sdf = spawn_proxy('bouncy_ball', sdf, "/",
                         pose, reference_frame)


  print 'Done.'
  while not rospy.is_shutdown():
    pass
  return


def externalCamera():
  sdf_path = '/home/julianalverio/catkin_ws/src/baxter_simulator/baxter_sim_examples/scripts/camera.sdf'
  sdf = open(sdf_path, 'r').read().replace('\n', '')
  rospy.wait_for_service('/gazebo/spawn_sdf_model')
  spawn_proxy = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
  index = 0
  pose = Pose()
  pose.position.x = 2.5
  pose.position.y = 0.0
  pose.position.z = 1.5
  pose.orientation.x = -0.15
  pose.orientation.y = 0
  pose.orientation.z = .98
  pose.orientation.w = .0

  resp_sdf = spawn_proxy('external_camera', sdf, "/",
                         pose, 'world')


def cleanUp(model_names=['bouncy_ball','test', 'table', 'external_camera', 'test1', 'test2', 'test3']):
# def cleanUp(model_names=['bouncy_ball', 'table']):
  delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
  for model_name in model_names:
    print 'attempting to delete: %s' % model_name
    try:
      delete_model(model_name)
      print 'Successful.'
    except:
      pass


def contactCallback(data):
  if data.states:
    print 'Contact detected!'
    print data.states


def nonantipodalGrasp():
  print 'Showing: Nonantipodal Grasp of Rectangular Prism.'

  pnp = PickAndPlace()
  pnp._gripper.set_holding_force(100)
  rospy.on_shutdown(cleanUp)

  print 'setting up camera...'
  externalCamera()

  pnp.gripper_open()

  print 'moving to start position...'
  pre_grip_angles = {'left_w0': 0.713473354262754, 'left_w1': 1.014095801262804, 'left_w2': -0.7107767620135959, 'left_e0': -0.598464939148772, 'left_e1': 0.9698857738523418, 'left_s0': -0.8576164362879198, 'left_s1': -0.2443509381144592}
  pnp._limb.move_to_joint_positions(pre_grip_angles)

  ##work table
  models = list()
  models.append(Model(name='table', shape='block', roll=0., pitch=0., yaw=0., restitution_coeff=0., size_x=.7, size_y=1.5, size_z=.7, x=.4, y=0.1, z=0., mass=5000, color='Grey', mu1=1, mu2=1, reference_frame=''))
  
  print 'generating table'
  loadGazeboModelsNoIO(models)

  models = []
  models.append(Model(name='test', shape='block', yaw=0.5, size_x=0.1, size_y=0.018, size_z=0.1, x=0.75/2, y=-0.00, z=0.3, mass=1, color='Yellow', mu1=1000, mu2=1000, reference_frame='world'))

  print 'generating test object'
  loadGazeboModelsNoIO(models)

  rospy.Subscriber("/l_side_r_finger_contact_sensor_state", ContactsState, contactCallback)
  rospy.Subscriber("/l_side_l_finger_contact_sensor_state", ContactsState, contactCallback)
  rospy.Subscriber("/r_side_r_finger_contact_sensor_state", ContactsState, contactCallback)
  rospy.Subscriber("/r_side_l_finger_contact_sensor_state", ContactsState, contactCallback)

  print 'Gripping now.'
  pnp.gripper_close()
  starting_overhead_angles = {'left_w0': 0.590693567655256, 'left_w1': 1.059736120576616, 'left_w2': -0.7595985440689625, 'left_e0': -1.1059240339618084, 'left_e1': 1.8919082513633318, 'left_s0': -0.1417684774660966, 'left_s1': -1.0218518327712416}
  pnp._limb.move_to_joint_positions(starting_overhead_angles)

  print 'Done. Hanging now.'
  while not rospy.is_shutdown():
    pass
  return 0


def pushObjectOnTable():
  print 'Pushing Rectangular Prism Into Table.'

  pnp = PickAndPlace()
  pnp._gripper.set_holding_force(100)
  rospy.on_shutdown(cleanUp)

  print 'setting up camera...'
  externalCamera()


  pnp.gripper_open()
  print 'moving to start position...'
  pre_grip_angles = {'left_w0': 0.713473354262754, 'left_w1': 1.014095801262804, 'left_w2': -0.7107767620135959, 'left_e0': -0.598464939148772, 'left_e1': 0.9698857738523418, 'left_s0': -0.8576164362879198, 'left_s1': -0.2443509381144592}
  pnp._limb.move_to_joint_positions(pre_grip_angles)


  ##work table
  models = list()
  models.append(Model(name='table', shape='block', roll=0., pitch=0., yaw=0., restitution_coeff=0., size_x=.7, size_y=1.5, size_z=.7, x=.4, y=0.1, z=0., mass=5000, color='Grey', mu1=1, mu2=1, reference_frame=''))
  
  print 'generating table'
  loadGazeboModelsNoIO(models)

  models = []
  models.append(Model(name='test', shape='block', yaw=0.0, size_x=0.1, size_y=0.018, size_z=0.1, x=0.75/2, y=-0.00, z=0.3, mass=1, color='Yellow', mu1=1000, mu2=1000, reference_frame='world'))

  print 'generating test object'
  loadGazeboModelsNoIO(models)

  rospy.Subscriber("/l_side_r_finger_contact_sensor_state", ContactsState, contactCallback)
  rospy.Subscriber("/l_side_l_finger_contact_sensor_state", ContactsState, contactCallback)
  rospy.Subscriber("/r_side_r_finger_contact_sensor_state", ContactsState, contactCallback)
  rospy.Subscriber("/r_side_l_finger_contact_sensor_state", ContactsState, contactCallback)

  print 'Gripping now.'
  pnp.gripper_close()

  current_pose = pnp._limb.endpoint_pose()
  next_pose = Pose()
  next_pose.position.x = current_pose['position'].x
  next_pose.position.y = current_pose['position'].y
  next_pose.position.z = current_pose['position'].z + 0.1
  next_pose.orientation = current_pose['orientation']

  angles = pnp.solveIK(next_pose)
  pnp._limb.move_to_joint_positions(angles)

  current_pose = pnp._limb.endpoint_pose()
  next_pose = Pose()
  next_pose.position.x = current_pose['position'].x
  next_pose.position.y = current_pose['position'].y
  next_pose.position.z = current_pose['position'].z - 0.2
  next_pose.orientation = current_pose['orientation']

  angles = pnp.solveIK(next_pose)
  pnp._limb.move_to_joint_positions(angles)

  #it should not get past this point.

  print 'Done. Hanging now.'
  while not rospy.is_shutdown():
    pass
  return 0


def pushHandOnTable():
  print 'Crashing Hand Into Table Top'

  pnp = PickAndPlace()
  pnp._gripper.set_holding_force(100)
  rospy.on_shutdown(cleanUp)

  print 'setting up camera...'
  externalCamera()

  print 'moving to start position...'

  pnp.gripper_close()

  pre_grip_angles = {'left_w0': 0.713473354262754, 'left_w1': 1.014095801262804, 'left_w2': -0.7107767620135959, 'left_e0': -0.598464939148772, 'left_e1': 0.9698857738523418, 'left_s0': -0.8576164362879198, 'left_s1': -0.2443509381144592}
  pnp._limb.move_to_joint_positions(pre_grip_angles)

  ##work table
  models = list()
  models.append(Model(name='table', shape='block', roll=0., pitch=0., yaw=0., restitution_coeff=0., size_x=.7, size_y=1.5, size_z=.7, x=.4, y=0.1, z=0., mass=5000, color='Grey', mu1=1, mu2=1, reference_frame=''))
  
  print 'generating table'
  loadGazeboModelsNoIO(models)

  rospy.Subscriber("/l_side_r_finger_contact_sensor_state", ContactsState, contactCallback)
  rospy.Subscriber("/l_side_l_finger_contact_sensor_state", ContactsState, contactCallback)
  rospy.Subscriber("/r_side_r_finger_contact_sensor_state", ContactsState, contactCallback)
  rospy.Subscriber("/r_side_l_finger_contact_sensor_state", ContactsState, contactCallback)

  current_pose = pnp._limb.endpoint_pose()
  next_pose = Pose()
  next_pose.position.x = current_pose['position'].x
  next_pose.position.y = current_pose['position'].y
  next_pose.position.z = current_pose['position'].z + 0.1
  next_pose.orientation = current_pose['orientation']

  angles = pnp.solveIK(next_pose)
  pnp._limb.move_to_joint_positions(angles)

  current_pose = pnp._limb.endpoint_pose()
  next_pose = Pose()
  next_pose.position.x = current_pose['position'].x
  next_pose.position.y = current_pose['position'].y
  next_pose.position.z = current_pose['position'].z - 0.2
  next_pose.orientation = current_pose['orientation']

  angles = pnp.solveIK(next_pose)
  pnp._limb.move_to_joint_positions(angles)

  #It should not get past this point.

  print 'Done. Hanging now.'
  while not rospy.is_shutdown():
    pass
  return 0


def grabCorners():
  print 'Grabbing Corners of Rectagular Prism.'

  pnp = PickAndPlace()
  pnp._gripper.set_holding_force(100)
  rospy.on_shutdown(cleanUp)

  print 'setting up camera...'
  externalCamera()

  print 'moving to start position...'

  
  pnp.gripper_open()

  pre_grip_angles = {'left_w0': 0.713473354262754, 'left_w1': 1.014095801262804, 'left_w2': -0.7107767620135959, 'left_e0': -0.598464939148772, 'left_e1': 0.9698857738523418, 'left_s0': -0.8576164362879198, 'left_s1': -0.2443509381144592}
  pnp._limb.move_to_joint_positions(pre_grip_angles)

  ##work table
  models = list()
  models.append(Model(name='table', shape='block', roll=0., pitch=0., yaw=0., restitution_coeff=0., size_x=.7, size_y=1.5, size_z=.7, x=.4, y=0.1, z=0., mass=5000, color='Grey', mu1=1, mu2=1, reference_frame=''))
  
  print 'generating table'
  loadGazeboModelsNoIO(models)

  models = []
  models.append(Model(name='test', shape='block', yaw=np.pi/4., size_x=0.02, size_y=0.018, size_z=0.1, x=0.75/2, y=-0.00, z=0.3, mass=1, color='Yellow', mu1=1000, mu2=1000, reference_frame='world'))

  print 'generating test object'
  loadGazeboModelsNoIO(models)

  rospy.Subscriber("/l_side_r_finger_contact_sensor_state", ContactsState, contactCallback)
  rospy.Subscriber("/l_side_l_finger_contact_sensor_state", ContactsState, contactCallback)
  rospy.Subscriber("/r_side_r_finger_contact_sensor_state", ContactsState, contactCallback)
  rospy.Subscriber("/r_side_l_finger_contact_sensor_state", ContactsState, contactCallback)

  print 'Gripping now.'
  pnp.gripper_close()

  current_pose = pnp._limb.endpoint_pose()
  next_pose = Pose()
  next_pose.position.x = current_pose['position'].x
  next_pose.position.y = current_pose['position'].y
  next_pose.position.z = current_pose['position'].z + 0.1
  next_pose.orientation = current_pose['orientation']

  angles = pnp.solveIK(next_pose)
  pnp._limb.move_to_joint_positions(angles)

  current_pose = pnp._limb.endpoint_pose()
  next_pose = Pose()
  next_pose.position.x = current_pose['position'].x
  next_pose.position.y = current_pose['position'].y
  next_pose.position.z = current_pose['position'].z - 0.2
  next_pose.orientation = current_pose['orientation']

  angles = pnp.solveIK(next_pose)
  pnp._limb.move_to_joint_positions(angles)

  #It should not get past this point.

  print 'Done. Hanging now.'
  while not rospy.is_shutdown():
    pass
  return 0


def manipulateCylinder():
  print 'Manipulating Cylinder.'

  pnp = PickAndPlace()
  pnp._gripper.set_holding_force(100)
  rospy.on_shutdown(cleanUp)

  print 'setting up camera...'
  externalCamera()

  print 'moving to start position...'
  pnp.gripper_open()

  pre_grip_angles = {'left_w0': 0.713473354262754, 'left_w1': 1.014095801262804, 'left_w2': -0.7107767620135959, 'left_e0': -0.598464939148772, 'left_e1': 0.9698857738523418, 'left_s0': -0.8576164362879198, 'left_s1': -0.2443509381144592}
  pnp._limb.move_to_joint_positions(pre_grip_angles)

  ##work table
  models = list()
  models.append(Model(name='table', shape='block', roll=0., pitch=0., yaw=0., restitution_coeff=0., size_x=.7, size_y=1.5, size_z=.7, x=.4, y=0.1, z=0., mass=5000, color='Grey', mu1=1, mu2=1, reference_frame=''))
  
  print 'generating table'
  loadGazeboModelsNoIO(models)

  models = []
  models.append(Model(name='test', shape='cylinder', yaw=0., size_r=0.018, size_z=0.12, x=0.75/2, y=-0.00, z=0.3, mass=1, color='Yellow', mu1=1000, mu2=1000, reference_frame='world'))

  print 'generating test object'
  loadGazeboModelsNoIO(models)

  rospy.Subscriber("/l_side_r_finger_contact_sensor_state", ContactsState, contactCallback)
  rospy.Subscriber("/l_side_l_finger_contact_sensor_state", ContactsState, contactCallback)
  rospy.Subscriber("/r_side_r_finger_contact_sensor_state", ContactsState, contactCallback)
  rospy.Subscriber("/r_side_l_finger_contact_sensor_state", ContactsState, contactCallback)

  print 'Gripping now.'
  pnp.gripper_close()

  current_pose = pnp._limb.endpoint_pose()
  next_pose = Pose()
  next_pose.position.x = current_pose['position'].x
  next_pose.position.y = current_pose['position'].y
  next_pose.position.z = current_pose['position'].z + 0.1
  next_pose.orientation = current_pose['orientation']

  angles = pnp.solveIK(next_pose)
  pnp._limb.move_to_joint_positions(angles)

  current_pose = pnp._limb.endpoint_pose()
  next_pose = Pose()
  next_pose.position.x = current_pose['position'].x
  next_pose.position.y = current_pose['position'].y
  next_pose.position.z = current_pose['position'].z - 0.2
  next_pose.orientation = current_pose['orientation']

  angles = pnp.solveIK(next_pose)
  pnp._limb.move_to_joint_positions(angles)

  print 'Done. Hanging now.'
  while not rospy.is_shutdown():
    pass
  return 0



def manipulateSphere():
  print 'Manipulating Sphere.'

  pnp = PickAndPlace()
  pnp._gripper.set_holding_force(100)
  rospy.on_shutdown(cleanUp)

  print 'setting up camera...'
  externalCamera()

  pnp.gripper_open()

  print 'moving to start position...'
  pre_grip_angles = {'left_w0': 0.713473354262754, 'left_w1': 1.014095801262804, 'left_w2': -0.7107767620135959, 'left_e0': -0.598464939148772, 'left_e1': 0.9698857738523418, 'left_s0': -0.8576164362879198, 'left_s1': -0.2443509381144592}
  pnp._limb.move_to_joint_positions(pre_grip_angles)

  ##work table
  models = list()
  models.append(Model(name='table', shape='block', roll=0., pitch=0., yaw=0., restitution_coeff=0., size_x=.7, size_y=1.5, size_z=.7, x=.4, y=0.1, z=0., mass=5000, color='Grey', mu1=1, mu2=1, reference_frame=''))
  
  print 'generating table'
  loadGazeboModelsNoIO(models)

  models = []
  models.append(Model(name='test', shape='sphere', yaw=0., size_r=0.02, x=0.75/2, y=0.005, z=0.3, mass=1, color='Yellow', mu1=1000, mu2=1000, reference_frame='world'))

  print 'generating test object'
  loadGazeboModelsNoIO(models)

  current_pose = pnp._limb.endpoint_pose()
  next_pose = Pose()
  next_pose.position.x = current_pose['position'].x
  next_pose.position.y = current_pose['position'].y
  next_pose.position.z = current_pose['position'].z - 0.05
  next_pose.orientation = current_pose['orientation']

  print 'doing IK for next movement...'
  next_angles = pnp.solveIK(next_pose)
  print 'moving to grasp'
  pnp._limb.move_to_joint_positions(next_angles)

  rospy.Subscriber("/l_side_r_finger_contact_sensor_state", ContactsState, contactCallback)
  rospy.Subscriber("/l_side_l_finger_contact_sensor_state", ContactsState, contactCallback)
  rospy.Subscriber("/r_side_r_finger_contact_sensor_state", ContactsState, contactCallback)
  rospy.Subscriber("/r_side_l_finger_contact_sensor_state", ContactsState, contactCallback)

  print 'Gripping now.'
  pnp.gripper_close()

  current_pose = pnp._limb.endpoint_pose()
  next_pose = Pose()
  next_pose.position.x = current_pose['position'].x
  next_pose.position.y = current_pose['position'].y
  next_pose.position.z = current_pose['position'].z + 0.1
  next_pose.orientation = current_pose['orientation']

  angles = pnp.solveIK(next_pose)
  pnp._limb.move_to_joint_positions(angles)

  current_pose = pnp._limb.endpoint_pose()
  next_pose = Pose()
  next_pose.position.x = current_pose['position'].x
  next_pose.position.y = current_pose['position'].y
  next_pose.position.z = current_pose['position'].z - 0.2
  next_pose.orientation = current_pose['orientation']

  angles = pnp.solveIK(next_pose)
  pnp._limb.move_to_joint_positions(angles)

  #It should not get past this point.

  print 'Done. Hanging now.'
  while not rospy.is_shutdown():
    pass

  return 0


def hitHandOnTableSide():
  print 'Colliding with Side of Table.'
  

  pnp = PickAndPlace()
  pnp._gripper.set_holding_force(100)
  rospy.on_shutdown(cleanUp)

  print 'setting up camera...'
  externalCamera()

  pnp.gripper_close()

  print 'moving to start position...'
  pre_grip_angles = {'left_w0': 0.713473354262754, 'left_w1': 1.014095801262804, 'left_w2': -0.7107767620135959, 'left_e0': -0.598464939148772, 'left_e1': 0.9698857738523418, 'left_s0': -0.8576164362879198, 'left_s1': -0.2443509381144592}
  pnp._limb.move_to_joint_positions(pre_grip_angles)

  ##work table
  models = list()
  models.append(Model(name='table', shape='block', roll=0., pitch=0., yaw=0., restitution_coeff=0., size_x=.7, size_y=1.5, size_z=.85, x=.4, y=0.45, z=0., mass=5000, color='Grey', mu1=1, mu2=1, reference_frame=''))
  
  print 'generating table'
  loadGazeboModelsNoIO(models)

  current_pose = pnp._limb.endpoint_pose()
  next_pose = Pose()
  next_pose.position.x = current_pose['position'].x
  next_pose.position.y = current_pose['position'].y + 0.3
  next_pose.position.z = current_pose['position'].z
  next_pose.orientation = current_pose['orientation']

  next_angles = pnp.solveIK(next_pose)
  pnp._limb.move_to_joint_positions(next_angles)

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


#samples_per = samples per trajectory execution (not actually executed lmao)
def collectData(num_samples=1000, ending_angles_dict=None, samples_per=8):
  path = '/home/julianalverio/catkin_ws/src/baxter_simulator/baxter_sim_examples/scripts/noisydata.csv'
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



def main():

  # starting_angles = {'left_w0': 0.6699952259595108,
  #                  'left_w1': 1.030009435085784,
  #                  'left_w2': -0.4999997247485215,
  #                  'left_e0': -1.189968899785275,
  #                  'left_e1': 1.9400238130755056,
  #                  'left_s0': -0.08000397926829805,
  #                  'left_s1': -0.9999781166910306}
  rospy.init_node("ik_pick_and_place_demo")
  rospy.wait_for_message("/robot/sim/started", Empty)
  pnp = PickAndPlace()

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
  # pnp._limb.move_to_joint_positions(zero_angles)
  # # pnp._limb.move_to_joint_positions(small_angles)
  # while not rospy.is_shutdown():
  #   pass
  # import sys; sys.exit()

  # rospy.init_node("ik_pick_and_place_demo")
  # rospy.wait_for_message("/robot/sim/started", Empty)
  # pnp = PickAndPlace()
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
      pnp._limb.move_to_joint_positions(move_angles)
  print 'NOW USING STANDARD PATH'
  start_dict = {'left_w0': 0.6699952259595108, 'left_w1': 1.030009435085784, 'left_w2': -0.4999997247485215,'left_e0': -1.189968899785275,'left_e1': 1.9400238130755056,'left_s0': -0.08000397926829805,'left_s1': -0.9999781166910306}
  goal_dict = {'left_w0': 0.713473354262754, 'left_w1': 1.014095801262804, 'left_w2': -0.7107767620135959, 'left_e0': -0.598464939148772, 'left_e1': 0.9698857738523418, 'left_s0': -0.8576164362879198, 'left_s1': -0.2443509381144592}
  pnp._limb.move_to_joint_positions(start_dict)

  pnp._limb.move_to_joint_positions(goal_dict)
  return



  pre_grip_angles = {'left_w0': 0.713473354262754, 'left_w1': 1.014095801262804, 'left_w2': -0.7107767620135959, 'left_e0': -0.598464939148772, 'left_e1': 0.9698857738523418, 'left_s0': -0.8576164362879198, 'left_s1': -0.2443509381144592}
  generateNoisyData(ending_angles_dict=pre_grip_angles)
  trajectories = getNoisyData()
  # pnp = PickAndPlace()
  # final_positions = [pair[1] for pair in trajectories]
  # for index, angles in enumerate(final_positions):
  #   print 'moving to', index
  #   pnp._limb.move_to_joint_positions(eval(angles))
  return

  pnp = PickAndPlace()
  pnp._limb.move_to_joint_positions(starting_angles)

  current_pose = pnp._limb.endpoint_pose()
  next_pose = Pose()
  next_pose.position.x = current_pose['position'].x + 0.1
  next_pose.position.y = current_pose['position'].y - 0.1
  next_pose.position.z = current_pose['position'].z + 0.1
  next_pose.orientation.x = current_pose['orientation'].x
  next_pose.orientation.y = current_pose['orientation'].y - 0.1
  next_pose.orientation.z = current_pose['orientation'].z - 0.1
  next_pose.orientation.w = current_pose['orientation'].w + 0.1
  next_angles = pnp.solveIK(next_pose)
  print "Showing Next Angles:"
  print next_angles
  pnp._limb.move_to_joint_positions(next_angles)

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
