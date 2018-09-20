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
import rospkg

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

  
class SceneController(object):
  def __init__(self):
    self.models = []
    self.camera = False
    self.srv = placeholder()
    self.pause_proxy = rospy.ServiceProxy('/gazebo/pause_physics', placeholder)
    self.unpause_proxy = rospy.ServiceProxy('/gazebo/unpause_physics', placeholder)
    self.scene_commander = moveit_commander.PlanningSceneInterface()

  # Pause the simulation
  def pause(self):
    self.pause_proxy()

  # Unpause the simulation
  def unpause(self):
    self.unpause_proxy()

  '''
  Spawn SDF models in Gazebo
  If moveit=True, also spawn the models in MoveIt! :D
  Input a list of Model Objects
  '''
  def spawnGazeboModels(self, models, moveit=False):
    self.models.extend(models)
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_proxy = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    for model in models:
      pose = model.pose
      reference_frame = model.reference_frame
      sdf = generateSDF(model, len(models)).replace('\n', '').replace('\t', '')
      try:
        print 'loading model named: %s' % model.name
        resp_sdf = spawn_proxy(model.name, sdf, "/",
                             pose, reference_frame)
        index += 1
      except rospy.ServiceException, e:
          rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    if moveit:
      for model in models:
        pose = Pose()
        pose.orientation = Quaternion(tf_conversions.transformations.quaternion_from_euler(model.roll, model.pitch, model.yaw))
        pose.position.x = model.x
        pose.position.y = model.y
        pose.position.z = model.z
        #TO BE CONTINUED

  # Recommended to be called upon ROS Exit. Deletes Gazebo models
  # Do not wait for the Gazebo Delete Model service, since
  # Gazebo should already be running. If the service is not
  # available since Gazebo has been killed, it is fine to error out
  # This will also delete any cameras you make
  def deleteGazeboModels(self):
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    except rospy.ServiceException, _:
        print 'FAILED TO CONTACT SERVICE PROXY: /gazebo/delete_model'

    for model in self.models:
      try:
        resp_delete = delete_model(model)
      except rospy.ServiceException, _:
        print 'FAILED TO DELETE MODEL: %s' % model
    if self.camera:
      delete_model('external_camera')

  #generate an external camera from camera.sdf
  #default values set intelligently
  def externalCamera(self, x=2.5, y=0.0, z=1.5, quat_x=-0.15, quat_y=0., quat_z=0.98, quat_w=0.):
    sdf = getSDFString.replace('\n', '').replace('\t', '')
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_proxy = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = quat_x
    pose.orientation.y = quat_y
    pose.orientation.z = quat_z
    pose.orientation.w = quat_w

    resp_sdf = spawn_proxy('external_camera', sdf, "/",
                           pose, 'world')
    self.camera = True


#Renders camera from camera.sdf
#To view camera output: 'rosrun image_view image_view image:=/cameras/external_camera/image'
#This cannot yet support multiple external cameras
class ExternalCamera(object):

  def __init__(self, x, y, z, roll, pitch, yaw):
    self.x = x
    self.y = y
    self.z = z
    self.roll = roll
    self.pitch = pitch
    self.yaw = yaw



  #Read in SDF string. See camera.sdf for an example of what will be generated.
  def getSDFString(self):
    sdf = ''
    sdf += '''<?xml version="1.0"?>\n'''
    sdf += '''<sdf version="1.6">\n'''
    sdf += '''\t<model name='camera'>\n'''
    sdf += '''\t\t<static>true</static>\n'''
    sdf += '''\t\t<pose frame=''>0 0 0 0 0 0</pose>\n'''
    sdf += '''\t\t<link name='link'>\n'''
    sdf += '''\t\t\t<visual name='visual'>\n'''
    sdf += '''\t\t\t\t<geometry>\n'''
    sdf += '''\t\t\t\t\t<box>\n'''
    sdf += '''\t\t\t\t\t\t<box>\n'''
    sdf += '''\t\t\t\t\t<box>\n'''
    sdf += '''\t\t\t\t<box>\n'''
    sdf += '''\t\t\t</visual>\n'''
    sdf += '''\t\t\t<sensor name='my_camera' type='camera'>\n'''
    sdf += '''\t\t\t\t<camera>\n'''
    sdf += '''\t\t\t\t\t<save enabled="true">\n'''
    sdf += '''\t\t\t\t\t\t<path>~/Documents/camera_save</path>\n'''
    sdf += '''\t\t\t\t\t</save>\n'''
    sdf += '''\t\t\t\t\t<horizontal_fov>1.047</horizontal_fov>\n'''
    sdf += '''\t\t\t\t\t<image>\n'''
    sdf += '''\t\t\t\t\t\t<width>1920</width>\n'''
    sdf += '''\t\t\t\t\t\t<height>1080</height>\n'''
    sdf += '''\t\t\t\t\t</image>\n'''
    sdf += '''\t\t\t\t\t<clip>\n'''
    sdf += '''\t\t\t\t\t\t<near>0.1</near>\n'''
    sdf += '''\t\t\t\t\t\t<far>100</far>\n'''
    sdf += '''\t\t\t\t\t</clip>\n'''
    sdf += '''\t\t\t\t</camera>\n'''
    sdf += '''\t\t\t\t<always_on>1</always_on>\n'''
    sdf += '''\t\t\t\t<update_rate>30</update_rate>\n'''
    sdf += '''\t\t\t\t<plugin name="camera_controller" filename="libgazebo_ros_camera.so">\n'''
    sdf += '''\t\t\t\t\t<alwaysOn>true</alwaysOn>\n'''
    sdf += '''\t\t\t\t\t<updateRate>0.0</updateRate>\n'''
    sdf += '''\t\t\t\t\t<cameraName>external_camera</cameraName>\n'''
    sdf += '''\t\t\t\t\t<imageTopicName>/cameras/external_camera/image</imageTopicName>\n'''
    sdf += '''\t\t\t\t\t<cameraInfoTopicName>/cameras/external_camera/camera_info</cameraInfoTopicName>\n'''
    sdf += '''\t\t\t\t\t<frameName>camera_frame</frameName>\n'''
    sdf += '''\t\t\t\t\t<hackBaseline>0.07</hackBaseline>\n'''
    sdf += '''\t\t\t\t\t<distortionK1>0.0</distortionK1>\n'''
    sdf += '''\t\t\t\t\t<distortionK2>0.0</distortionK2>\n'''
    sdf += '''\t\t\t\t\t <distortionK3>0.0</distortionK3>\n'''
    sdf += '''\t\t\t\t\t<distortionT1>0.0</distortionT1>\n'''
    sdf += '''\t\t\t\t\t<distortionT2>0.0</distortionT2>\n'''
    sdf += '''\t\t\t\t</plugin>\n'''
    sdf += '''\t\t\t</sensor>\n'''
    sdf += '''\t\t</link>\n'''
    sdf += '''\t</model>\n'''
    sdf ++ '''</sdf>'''
    return sdf


class Model(object):
  def __init__(self, shape='box', size_x=0.5, size_y=0.5, 
               size_z=0.5, size_r=0.5, x=None, y=None, z=None, 
               mass=0.5, color=None, mu1=1000, mu2=1000,
               reference_frame='world',
               restitution_coeff=0.5, roll=0., pitch=0., yaw=0.,
               name=None):
    self.shape = shape
    if self.shape not in ['box', 'cylinder', 'sphere']:
      self.shape = 'box'
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
  #assumption: mass is uniformly distributed
  #reference: dynref.engr.illinois.edu/rem.html
  #returns: [ixx, iyy, izz]
  def getMoments(self):
    moments = np.ones((1, 3))
    if self.shape == 'box':
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

def generateGeometrySDF(model):
  sdf = '\t\t\t\t<geometry>\n'
  sdf += '\t\t\t\t\t<%s>\n' % model.shape
  if model.shape == 'sphere':
    sdf += '\t\t\t\t\t\t<radius>%s</radius>\n' % model.size_r
  if model.shape == 'cylinder':
    sdf += '\t\t\t\t\t\t<radius>%s</radius>\n' % model.size_r
    sdf += '\t\t\t\t\t\t<height>%s</height>\n' % model.size_z
  if model.shape == 'box':
    sdf += '\t\t\t\t\t\t<size>%s %s %s</size>\n' % (model.size_x, model.size_y, model.size_z)
  sdf += '\t\t\t\t</geometry>\n'
  return sdf


#Takes in a Model object
#Returns SDF string
def generateSDFExperimental(model, index=0, writeToFile=False):
  sdf = ''
  if not model.name:
    name = 'object%d' % index
    model.name = name
  else:
    name = model.name
  sdf += '<?xml version="1.0" ?>\n'
  sdf += '<sdf version="1.3">\n'
  sdf += '\t<model name="%s">\n' % name
  sdf += '\t\t<pose>%s %s %s %s %s %s</pose>\n' % (model.x, model.y, model.z, model.roll, model.pitch, model.yaw)
  sdf += '\t\t<link name="link_%s">\n' % model.name
  sdf += '\t\t\t<inertial>\n'
  sdf += '\t\t\t\t<origin xyz="%f %f %f" rpy="%f %f %f" />\n' % (model.x, model.y, model.z, model.roll, model.pitch, model.yaw)
  sdf += '\t\t\t\t<mass value="%s" />\n' % model.mass
  sdf += '\t\t\t\t<inertia  ixx="%f" ixy="%f"  ixz="%f"  iyy="%f"  iyz="%f"  izz="%f" />\n' % (model.ixx, model.ixy, model.ixz, model.iyy, model.iyz, model.izz)
  sdf += '\t\t\t</inertial>\n'
  sdf += '\t\t\t<collision name="collision_%s">\n' % model.name
  sdf += generateGeometrySDF(model)
  sdf += '\t\t\t\t</geometry>\n'
  sdf += '\t\t\t\t<surface>\n'
  sdf += '\t\t\t\t\t<bounce>\n'
  sdf += '\t\t\t\t\t\t<restitution_coefficient>%d</restitution_coefficient>\n' % model.COR
  sdf += '\t\t\t\t\t\t<threshold>0</threshold>\n'
  sdf += '\t\t\t\t\t</bounce>\n'
  sdf += '\t\t\t\t\t<contact>\n'
  sdf += '\t\t\t\t\t\t<ode>\n'
  sdf += '\t\t\t\t\t\t\t<max_vel>10000</max_vel>\n'
  sdf += '\t\t\t\t\t\t</ode>\n'
  sdf += '\t\t\t\t\t</contact>\n'
  sdf += '\t\t\t\t</surface>\n'
  sdf += '\t\t\t</collision>\n'
  sdf += '\t\t\t<visual name="visual_%s"\n' % model.name
  sdf += generateGeometrySDF(model)
  sdf += '\t\t\t\t<material>\n'
  sdf += '\t<gazebo reference="%s">\n' % name
  sdf += '\t\t<material>Gazebo/%s</material>\n' % model.color
  sdf += '\t\t\t<mu1>%f</mu1>\n' % model.mu1
  sdf += '\t\t\t<mu2>%f</mu1>\n' % model.mu2
  sdf += '\t</gazebo>\n'
  if writeToFile:
    with open('%s.sdf', 'w+') as f:
      f.write(sdf)
  return sdf



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
  if model.shape == 'box':
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
  if model.shape == 'box':
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
