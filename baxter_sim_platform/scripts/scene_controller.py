#!/usr/bin/env python

import random
import numpy as np
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

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

from std_srvs.srv import Empty as placeholder


  
class SceneController(object):
  def __init__(self):
    self.models = []
    self.cameras = 0
    self.srv = placeholder()
    self.pause_proxy = rospy.ServiceProxy('/gazebo/pause_physics', placeholder)
    self.unpause_proxy = rospy.ServiceProxy('/gazebo/unpause_physics', placeholder)
    self.scene_commander = moveit_commander.PlanningSceneInterface()

  def shutdown(self):
    self.deleteAllGazeboModels()
    self.deleteAllMoveItModels()

####################################################################################################
################################# Pause/Unpause Simulation Methods #################################
####################################################################################################

  # Pause the simulation
  def pause(self):
    self.pause_proxy()

  # Unpause the simulation
  def unpause(self):
    self.unpause_proxy()

####################################################################################################
############################### Methods to Add/Remove Gazebo Objects ###############################
####################################################################################################

  '''
  Spawn SDF models in Gazebo
  If moveit=True, also spawn the models in MoveIt! :D
  NOTE: This method only supports BOXES and SPHERES. It can be extended to support meshes.
  NOTE: MoveIt! Currently only supports boxes, spheres, and meshes -- NOT cylinders.
  http://docs.ros.org/jade/api/moveit_commander/html/planning__scene__interface_8py_source.html 
  '''
  def spawnGazeboModels(self, moveit=False):
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_proxy = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    for model in self.models:
      pose = model.pose
      reference_frame = model.reference_frame
      sdf = model.generateSDF().replace('\n', '').replace('\t', '')
      try:
        rospy.loginfo('Loading model named: %s' % model.name)
        resp_sdf = spawn_proxy(model.name, sdf, "/",
                             pose, reference_frame)
        print("Success!")
      except rospy.ServiceException, e:
          rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    if moveit:
      for model in models:
        pose = Pose()
        pose.orientation = Quaternion(
          tf_conversions.transformations.quaternion_from_euler(model.roll, model.pitch, model.yaw))
        pose.position.x = model.x
        pose.position.y = model.y
        pose.position.z = model.z
        model_msg = geometry_msgs.msg.PoseStamped()
        model_msg.pose = pose
        if model.shape == 'box':
          self.scene_commander.add_box(
            model.name, pose, size=(model.size_x, model.size_y, model.size_z))
          success = waitForSpawn(model.name)
        if model.shape == 'sphere':
          self.scene_commander.add_sphere(model.name, pose, radius=model.size_r)
        else:
          rospy.logerr('MoveIt is ignoring model named %s of shape %s.' % (model.name, model.shape))

  # Similar to spawnGazeboModels, but for only one model for which you pass in the name
  def spawnGazeboModel(self, model_name, moveit=False):
    for _model in self.models:
      if _model.name == model_name:
        model = _model
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_proxy = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    pose = model.pose
    reference_frame = model.reference_frame
    sdf = model.generateSDF().replace('\n', '').replace('\t', '')
    try:
      rospy.loginfo('Loading model named: %s...' % model.name)
      resp_sdf = spawn_proxy(model.name, sdf, "/",
                           pose, reference_frame)
      rospy.loginfo(resp_sdf)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    if moveit:
      pose = Pose()
      pose.orientation = Quaternion(
        tf_conversions.transformations.quaternion_from_euler(model.roll, model.pitch, model.yaw))
      pose.position.x = model.x
      pose.position.y = model.y
      pose.position.z = model.z
      model_msg = geometry_msgs.msg.PoseStamped()
      model_msg.pose = pose
      if model.shape == 'box':
        self.scene_commander.add_box(
          model.name, pose, size=(model.size_x, model.size_y, model.size_z))
        success = waitForSpawn(model.name)
      if model.shape == 'sphere':
        self.scene_commander.add_sphere(model.name, pose, radius=model.size_r)
      else:
        rospy.logerr('MoveIt is ignoring model named %s of shape %s.' % (model.name, model.shape))


  '''
  Sometimes a node can fail and a message will be lost, causing an object to not to what's expected
  Wait for the appropriate action to happen and ensure nothing gets lost.
  Inputs:
    name: name of the object you're waiting for
    end: must be 'scene' or 'attached' -- the end state you want to object to be in
    timeout: how long you're willing to wait for the object
  '''
  def waitForMoveItObject(self, name, end='scene', timeout=5):
    start = rospy.get_time()
    while (rospy.get_time() - start < timeout) and not rospy.is_shutdown():
      in_scene = name in scene.get_known_object_names()
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      if in_scene and is_attached:
        rospy.logerr('Aww snap. An object was found both in the scene and attached to the arm.')
        rospy.shutdown()

      if end == 'scene' and in_scene:
        return

      if end == 'attached' and is_attached:
        return

    rospy.logerr('Yikes -- timed out while adding object to MoveIt! scene. A node must have failed')
    rospy.shutdown()


  '''
  Will be called upon ROS Exit. Deletes Gazebo models
  Do not wait for the Gazebo Delete Model service, since
  Gazebo should already be running. If the service is not
  available since Gazebo has been killed, it is fine to error out
  This will also delete any cameras you make
  '''
  def deleteAllGazeboModels(self):
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    except rospy.ServiceException, _:
        print 'FAILED TO CONTACT SERVICE PROXY: /gazebo/delete_model'

    for model in self.models:
      try:
        resp_delete = delete_model(model.name)
      except rospy.ServiceException, _:
        print 'FAILED TO DELETE MODEL: %s' % model.name
    for camera_idx in xrange(self.cameras):
      camera_name = 'camera_' + str(camera_idx)
      try:
        resp_delete = delete_model(camera_name)
      except rospy.ServiceException, _:
        print 'FAILED TO DELETE CAMERA: %s' % camera_name
    self.cameras = 0
    self.models = []


  # Delete a particular Gazebo model
  def deleteGazeboModel(self, model_name):
    try:
      delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    except rospy.ServiceException, _:
      print 'FAILED TO CONTACT SERVICE PROXY: /gazebo/delete_model'
    try:
      resp_delete = delete_model(model.name)
    except rospy.ServiceException, _:
      print 'FAILED TO DELETE MODEL: %s' % model.name
    for idx, model in self.models:
      if  model.name == model_name:
        self.models.pop(idx)


  # Deletes a particular MoveIt object
  def deleteMoveItModel(self, object_name):
    scene_commander.remove_world_object(object_name)


  # Deletes all MoveIt! objects
  def deleteAllMoveItModels(self):
    for object_name in [model.name for model in self.models]:
      self.scene_commander.remove_world_object(object_name)
  

  def makeModel(self, shape='box', size_x=0.5, size_y=0.5, 
               size_z=0.5, size_r=0.5, x=None, y=None, z=None, 
               mass=0.5, ambient_r=0, ambient_g=1, ambient_b=0, ambient_a=1, mu1=1000, mu2=1000,
               reference_frame='world',
               restitution_coeff=0.5, roll=0., pitch=0., yaw=0.,
               name=None, diffuse_r=1, diffuse_g=1, diffuse_b=1, diffuse_a=0):
    if not name:
      name = 'object_' + str(len(self.models))
    model = Model(shape=shape, size_x=size_x, size_y=size_y,
      size_z=size_z, size_r=size_r, x=x, y=y, z=z, mass=mass, ambient_r=ambient_r, ambient_g=ambient_g,
      ambient_b=ambient_b, ambient_a=ambient_a, mu1=mu1, mu2=mu2, reference_frame=reference_frame, 
      restitution_coeff=restitution_coeff, roll=roll, pitch=pitch, yaw=yaw, name=name, diffuse_r=diffuse_r,
      diffuse_g=diffuse_g, diffuse_b=diffuse_b, diffuse_a=diffuse_a)
    self.models.append(model)
    self.checkUniqueModelNames()
    return model


  '''
  Check that all model objects in the scene have unique names
  Throw an assertion error if not
  Hint: If you leave the name fields blank, unique names will be generated for you
  '''
  def checkUniqueModelNames(self):
    names = []
    for model in self.models:
      names.append(model.name)
    assert len(names) == len(set(names)), 'Model Names Must Be Unique!'


####################################################################################################
###################################### Camera-Related Methods ######################################
####################################################################################################

  '''
  Generate and render an external camera to view the scene
  Default values are set intelligently to view the Baxter.
  Inputs: location/orientation parameters for camera placement
  NOTE: This platform can support any number of external cameras
  WARNING: External cameras are computationally expensive
  Camera names are automatically generated as camera_0, camera_1, etc.
  To view camera output: 'rosrun image_view image_view image:=/cameras/CAMERA_NAME/image'
  '''
  def externalCamera(self, x=2.5, y=0.0, z=1.5, quat_x=-0.15, quat_y=0., quat_z=0.98, quat_w=0.):
    camera = ExternalCamera(x, y, z, quat_x, quat_y, quat_z, quat_w, self.cameras)
    self.cameras += 1
    camera.render()


####################################################################################################
####################################### External Camera Class ######################################
####################################################################################################

class ExternalCamera(object):

  def __init__(self, x, y, z, quat_x, quat_y, quat_z, quat_w, camera_count):
    self.x = x
    self.y = y
    self.z = z
    self.quat_x = quat_x
    self.quat_y = quat_y
    self.quat_z = quat_z
    self.quat_w = quat_w
    self.name = 'camera_' + str(camera_count)


  def render(self):
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_proxy = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    pose = Pose()
    pose.position.x = self.x
    pose.position.y = self.y
    pose.position.z = self.z
    pose.orientation.x = self.quat_x
    pose.orientation.y = self.quat_y
    pose.orientation.z = self.quat_z
    pose.orientation.w = self.quat_w

    resp_sdf = spawn_proxy(self.name, self.getSDFString(), "/",
                           pose, '')


  #Read in SDF string. See camera.sdf for an example of what will be generated.
  def getSDFString(self):
    sdf = ''
    sdf += '<?xml version="1.0"?>'
    sdf += '<sdf version="1.6">'
    sdf += '\t<model name="%s">' % self.name
    sdf += '\t\t<static>true</static>'
    sdf += '<pose frame=''>0 0 0 0 0 0</pose>'
    sdf += '<link name="%s_link">' % self.name
    sdf += '\t\t\t<visual name="%s_visual">' % self.name
    sdf += '\t\t\t\t<geometry>'
    sdf += '\t\t\t\t\t<box>'
    sdf += '\t\t\t\t\t\t<size>0.1 0.1 0.1</size>'
    sdf += '\t\t\t\t\t</box>'
    sdf += '\t\t\t\t</geometry>'
    sdf += '\t\t\t</visual>'
    sdf += '\t\t\t<sensor name="%s" type="camera">' % self.name
    # sdf + '\t\t\t\t<camera>'
    # sdf += '\t\t\t\t\t<save enabled="true">'
    # sdf += '\t\t\t\t\t\t<path>~/catkin_ws/src/baxter_platform/baxter_sim_platform/images/camera_save</path>'
    # sdf += '\t\t\t\t\t</save>'
    # sdf += '\t\t\t\t\t<image>'
    # sdf += '\t\t\t\t\t\t<width>1920</width>'
    # sdf += '\t\t\t\t\t\t<height>1080</height>'
    # sdf += '\t\t\t\t\t</image>'
    # sdf += '\t\t\t\t</camera>'
    sdf += '\t\t\t\t<always_on>1</always_on>'
    sdf += '\t\t\t\t<update_rate>30</update_rate>'
    sdf += '\t\t\t\t<plugin name="camera_controller" filename="libgazebo_ros_camera.so">'
    sdf += '\t\t\t\t\t<alwaysOn>true</alwaysOn>'
    sdf += '\t\t\t\t\t<updateRate>0.0</updateRate>'
    sdf += '\t\t\t\t\t<cameraName>%s</cameraName>' % self.name
    sdf += '\t\t\t\t\t<imageTopicName>/cameras/%s/image</imageTopicName>' % self.name
    sdf += '\t\t\t\t\t<cameraInfoTopicName>/cameras/%s/camera_info</cameraInfoTopicName>' % self.name
    sdf += '\t\t\t\t\t<frameName>%s_frame</frameName>' % self.name
    sdf += '\t\t\t\t\t<hackBaseline>0.07</hackBaseline>'
    sdf += '\t\t\t\t\t<distortionK1>0.0</distortionK1>'
    sdf += '\t\t\t\t\t<distortionK2>0.0</distortionK2>'
    sdf += '\t\t\t\t\t<distortionK3>0.0</distortionK3>'
    sdf += '\t\t\t\t\t<distortionT1>0.0</distortionT1>'
    sdf += '\t\t\t\t\t<distortionT2>0.0</distortionT2>'
    sdf += '\t\t\t\t</plugin>'
    sdf += '\t\t\t</sensor>'
    sdf += '\t\t</link>'
    sdf += '\t</model>'
    sdf += '</sdf>'
    return sdf


####################################################################################################
############################################ Model Class ###########################################
####################################################################################################

class Model(object):
  def __init__(self, shape='box', size_x=0.5, size_y=0.5, size_z=0.5, size_r=0.5, x=None, y=None, 
               z=None, mass=0.5, ambient_r=0, ambient_g=1, ambient_b=0, ambient_a=0, mu1=1000, mu2=1000,
               reference_frame='world', restitution_coeff=0.5, roll=0., pitch=0., yaw=0.,
               name=None, diffuse_r=1, diffuse_g=1, diffuse_b=1, diffuse_a=0):
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
    self.ambient_r = ambient_r
    self.ambient_g = ambient_g
    self.ambient_b = ambient_b
    self.ambient_a = ambient_a
    self.diffuse_r = diffuse_r
    self.diffuse_g = diffuse_g
    self.diffuse_b = diffuse_b
    self.diffuse_a = diffuse_a
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

  '''
  Calculates moments: [ixx, iyy, izz]
  Assumes mass is uniformly distributed
  Reference: dynref.engr.illinois.edu/rem.html
  '''
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

  #Returns SDF representation of a model
  def generateSDF(self):
    sdf = ''
    sdf += '<?xml version="1.0"?>\n'
    sdf += '<sdf version="1.6">\n'
    sdf += '\t<model name="%s">\n' % self.name
    sdf += '\t\t<pose>%s %s %s %s %s %s</pose>\n' % (self.x, self.y, self.z, self.roll, self.pitch, self.yaw)
    sdf += '\t\t<link name="%s_link">\n' % self.name
    sdf += '\t\t\t<inertial>\n'
    sdf += '\t\t\t\t<mass>%s</mass>\n' % self.mass
    sdf += '\t\t\t\t<inertia>\n'
    sdf += '\t\t\t\t\t<ixx>%s</ixx>\n' % self.ixx
    sdf += '\t\t\t\t\t<ixy>%s</ixy>\n' % self.ixy
    sdf += '\t\t\t\t\t<ixz>%s</ixz>\n' % self.ixz
    sdf += '\t\t\t\t\t<iyy>%s</iyy>\n' % self.iyy
    sdf += '\t\t\t\t\t<iyz>%s</iyz>\n' % self.iyz
    sdf += '\t\t\t\t\t<izz>%s</izz>\n' % self.izz
    sdf += '\t\t\t\t</inertia>\n'
    sdf += '\t\t\t</inertial>\n'
    sdf += '\t\t\t<collision name="%s_collision">\n' % self.name
    sdf += self.generateGeometrySDF()
    sdf += '\t\t\t\t<surface>\n'
    sdf += '\t\t\t\t\t<bounce>\n'
    sdf += '\t\t\t\t\t\t<restitution_coefficient>%s</restitution_coefficient>\n' % self.COR
    sdf += '\t\t\t\t\t\t<threshold>0</threshold>\n'
    sdf += '\t\t\t\t\t</bounce>\n'
    sdf += '\t\t\t\t\t<contact>\n'
    sdf += '\t\t\t\t\t\t<ode>\n'
    sdf += '\t\t\t\t\t\t\t<max_vel>10000</max_vel>\n'
    sdf += '\t\t\t\t\t\t</ode>\n'
    sdf += '\t\t\t\t\t</contact>\n'
    sdf += '\t\t\t\t\t<friction>\n'
    sdf += '\t\t\t\t\t\t<ode>\n'
    sdf += '\t\t\t\t\t\t\t<mu>%s</mu>\n' % self.mu1
    sdf += '\t\t\t\t\t\t\t<mu2>%s</mu2>\n' % self.mu2
    sdf += '\t\t\t\t\t\t</ode>\n'
    sdf += '\t\t\t\t\t</friction>\n'
    sdf += '\t\t\t\t</surface>\n'
    sdf += '\t\t\t</collision>\n'
    sdf += '\t\t\t<visual name="%s_visual">\n' % self.name
    sdf += self.generateGeometrySDF()
    sdf += '\t\t\t\t<material>\n'
    sdf += '\t\t\t\t\t<ambient> %s %s %s %s</ambient>\n' % (self.ambient_r, self.ambient_g, self.ambient_b, self.ambient_a)
    sdf += '\t\t\t\t\t<diffuse> %s %s %s %s</diffuse>\n' % (self.diffuse_r, self.diffuse_g, self.diffuse_b, self.diffuse_a)
    sdf += '\t\t\t\t</material>\n'
    sdf += '\t\t\t</visual>\n'
    sdf += '\t\t</link>\n'
    sdf += '\t</model>\n'
    sdf += '</sdf>'
    f = open('/home/infolab/catkin_ws/src/baxter_platform/baxter_sim_platform/models/delete_later.sdf', 'w+')
    f.write(sdf)
    return sdf

  # Helper method called by generateSDF()
  def generateGeometrySDF(self):
    sdf = '\t\t\t\t<geometry>\n'
    sdf += '\t\t\t\t\t<%s>\n' % self.shape
    if self.shape == 'sphere':
      sdf += '\t\t\t\t\t\t<radius>%s</radius>\n' % self.size_r
    if self.shape == 'cylinder':
      sdf += '\t\t\t\t\t\t<radius>%s</radius>\n' % self.size_r
      sdf += '\t\t\t\t\t\t<height>%s</height>\n' % self.size_z
    if self.shape == 'box':
      sdf += '\t\t\t\t\t\t<size>%s %s %s</size>\n' % (self.size_x, self.size_y, self.size_z)
    sdf += '\t\t\t\t\t</%s>\n' % self.shape
    sdf += '\t\t\t\t</geometry>\n'
    return sdf