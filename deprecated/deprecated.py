#old camera generation sdf code
# sdf = ''
# sdf += '<?xml version="1.0"?>\n'
# sdf += '<sdf version="1.6">\n'
# sdf += '\t<model name="%s">\n' % self.name
# sdf += '\t\t<static>true</static>\n'
# sdf += '\t\t<pose frame=''>0 0 0 0 0 0</pose>\n'
# sdf += '\t\t<link name="%s_link">\n' % self.name
# sdf += '\t\t\t<visual name='%s_visual'>\n' % self.name
# sdf += '\t\t\t\t<geometry>\n'
# sdf += '\t\t\t\t\t<box>\n'
# sdf += '''\t\t\t\t\t\t<box>\n'''
# sdf += '''\t\t\t\t\t<box>\n'''
# sdf += '''\t\t\t\t<box>\n'''
# sdf += '''\t\t\t</visual>\n'''
# sdf += '''\t\t\t<sensor name='my_camera' type='camera'>\n'''
# sdf += '''\t\t\t\t<camera>\n'''
# sdf += '''\t\t\t\t\t<save enabled="true">\n'''
# sdf += '''\t\t\t\t\t\t<path>~/Documents/camera_save</path>\n'''
# sdf += '''\t\t\t\t\t</save>\n'''
# sdf += '''\t\t\t\t\t<horizontal_fov>1.047</horizontal_fov>\n'''
# sdf += '''\t\t\t\t\t<image>\n'''
# sdf += '''\t\t\t\t\t\t<width>1920</width>\n'''
# sdf += '''\t\t\t\t\t\t<height>1080</height>\n'''
# sdf += '''\t\t\t\t\t</image>\n'''
# sdf += '''\t\t\t\t\t<clip>\n'''
# sdf += '''\t\t\t\t\t\t<near>0.1</near>\n'''
# sdf += '''\t\t\t\t\t\t<far>100</far>\n'''
# sdf += '''\t\t\t\t\t</clip>\n'''
# sdf += '''\t\t\t\t</camera>\n'''
# sdf += '''\t\t\t\t<always_on>1</always_on>\n'''
# sdf += '''\t\t\t\t<update_rate>30</update_rate>\n'''
# sdf += '''\t\t\t\t<plugin name="camera_controller" filename="libgazebo_ros_camera.so">\n'''
# sdf += '''\t\t\t\t\t<alwaysOn>true</alwaysOn>\n'''
# sdf += '''\t\t\t\t\t<updateRate>0.0</updateRate>\n'''
# sdf += '''\t\t\t\t\t<cameraName>%s</cameraName>\n''' % self.name
# sdf += '''\t\t\t\t\t<imageTopicName>/cameras/%s/image</imageTopicName>\n''' % self.name
# sdf += '''\t\t\t\t\t<cameraInfoTopicName>/cameras/%s/camera_info</cameraInfoTopicName>\n''' % self.name
# sdf += '''\t\t\t\t\t<frameName>camera_frame</frameName>\n'''
# sdf += '''\t\t\t\t\t<hackBaseline>0.07</hackBaseline>\n'''
# sdf += '''\t\t\t\t\t<distortionK1>0.0</distortionK1>\n'''
# sdf += '''\t\t\t\t\t<distortionK2>0.0</distortionK2>\n'''
# sdf += '''\t\t\t\t\t <distortionK3>0.0</distortionK3>\n'''
# sdf += '''\t\t\t\t\t<distortionT1>0.0</distortionT1>\n'''
# sdf += '''\t\t\t\t\t<distortionT2>0.0</distortionT2>\n'''
# sdf += '''\t\t\t\t</plugin>\n'''
# sdf += '''\t\t\t</sensor>\n'''
# sdf += '''\t\t</link>\n'''
sdf += '''\t</model>\n'''
sdf += '''</sdf>'''
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
  robot_controller = RobotController('left')
  robot_controller.followTrajectoryFromJointVelocity(vels)


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
  robot_controller = RobotController('left')
  trajectory = list()
  angles = [-0.11, -0.62, -1.15, 1.32,  0.80, 1.27,  2.39]
  trajectory.append((angles, 5.0))
  angles = [angle*0.5 for angle in angles]
  trajectory.append((angles, 5.0))
  angles = [angle*1.75 for angle in angles]
  trajectory.append((angles, 5.0))
  robot_controller.followTrajectoryWithIK(trajectory)

#deprecated
def testFollowTrajectoryFromJointAngles():
  rospy.init_node("joint_angle_follow")
  rospy.wait_for_message("/robot/sim/started", Empty)

  robot_controller = RobotController('left')
  starting_joint_angles = {'left_w0': 0.6699952259595108,
                       'left_w1': 1.030009435085784,
                       'left_w2': -0.4999997247485215,
                       'left_e0': -1.189968899785275,
                       'left_e1': 1.9400238130755056,
                       'left_s0': -0.08000397926829805,
                       'left_s1': -0.9999781166910306}
  robot_controller._limb.move_to_joint_positions(starting_joint_angles)

  #this trajectory was generated with the IK solver
  trajectory = [{'left_w0': 0.6663439517797247, 'left_w1': 1.0254832967958967, 'left_w2': -0.49791594416740903, 'left_e0': -1.1847989726061723, 'left_e1': 1.933762430285121, 'left_s0': -0.07890395814907691, 'left_s1': -0.9913289297553849}, {'left_w0': 0.7353491779669039, 'left_w1': 1.3090406232622902, 'left_w2': -0.21668752438391392, 'left_e0': -0.9407004398680272, 'left_e1': 1.2076700198856085, 'left_s0': -0.390958749829128, 'left_s1': -0.6675685155625416}, {'left_w0': 0.7330713167542067, 'left_w1': 1.3162117949266057, 'left_w2': 0.07355005285647863, 'left_e0': -0.929507737190373, 'left_e1': 1.166775364272026, 'left_s0': -0.12895727189225206, 'left_s1': -0.6402246273083545}, {'left_w0': 0.6662213273580528, 'left_w1': 1.7333649961627016, 'left_w2': 0.19624972970590862, 'left_e0': -0.9873696310196056, 'left_e1': 0.8023198330522042, 'left_s0': -0.20288995772020885, 'left_s1': -0.7783856313894464}, {'left_w0': 0.699089074999042, 'left_w1': 1.7495692436937351, 'left_w2': 0.5839992707054027, 'left_e0': -1.0308852381232938, 'left_e1': 0.7698219339384003, 'left_s0': -0.19511293413759315, 'left_s1': -0.7510437605072855}, {'left_w0': 0.6971546624084283, 'left_w1': 1.7674929011830742, 'left_w2': 0.4910183488567289, 'left_e0': -1.0107976354711212, 'left_e1': 0.7418040910707417, 'left_s0': -0.216089350524972, 'left_s1': -0.7452487873887369}, {'left_w0': 0.8543116593525074, 'left_w1': 1.595139000726061, 'left_w2': 0.27291051124465476, 'left_e0': -0.9543491138939362, 'left_e1': 1.0820713288402664, 'left_s0': -0.27074366544625333, 'left_s1': -0.9141157077556324}, {'left_w0': 0.7541181630688458, 'left_w1': 1.0843403328865067, 'left_w2': 0.13452227613101395, 'left_e0': -1.147813327587313, 'left_e1': 1.560391127177694, 'left_s0': 0.09741270782365391, 'left_s1': -1.1596446589326543}]
  robot_controller.followTrajectoryFromJointAngles(trajectory)
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
  robot_controller._limb.move_to_joint_positions(starting_joint_angles)
  robot_controller.followTrajectoryFromJointAngles(second_trajectory)



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


  '''
  Attach box to end effector to simulate picking it up.
  Note: This method only supports picking up boxes.
  Note: MoveIt! Only supports picking up boxes and meshes.
  '''
  def attachBox(self, end_effector_link, box_name, side='left'):
    grasping_group = side + '_gripper'
    touch_links = robot_commander.get_link_names(group=grasping_group)
    scene_commander.attach_box(end_effector_link, box_name, touch_links=touch_links)

  '''
  Detach box from end effector to simulate dropping it.
  Note: This method only supports boxes.
  '''
  def detachBox(self, end_effector_link, box_name):
    scene_commander.remove_attached_object(end_effector_link, name=box_name)

