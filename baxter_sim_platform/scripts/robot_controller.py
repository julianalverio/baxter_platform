#!/usr/bin/env python

import struct
import sys
import copy
import numpy as np
import rospy
import genpy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import baxter_interface
from baxter_interface import CHECK_VERSION
import os
import rospkg
from trajectory import *

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from std_msgs.msg import (
    Header,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from baxter_core_msgs.msg import (
    JointCommand,
)
from gazebo_msgs.msg import ContactsState




class RobotController(object):
  def __init__(self, tolerance=0.008726646):
    baxter_interface.JOINT_ANGLE_TOLERANCE = tolerance
    self._left_limb = baxter_interface.Limb('left')
    self._right_limb = baxter_interface.Limb('right')
    self._left_gripper = baxter_interface.Gripper('left')
    self._right_gripper = baxter_interface.Gripper('right')
    left_ns = "ExternalTools/" + 'left' + "/PositionKinematicsNode/IKService"
    self.left_iksvc = rospy.ServiceProxy(left_ns, SolvePositionIK)
    rospy.wait_for_service(left_ns, 5.0)
    right_ns = "ExternalTools/" + 'right' + "/PositionKinematicsNode/IKService"
    self.right_iksvc = rospy.ServiceProxy(right_ns, SolvePositionIK)
    rospy.wait_for_service(right_ns, 5.0)

    # verify robot is enabled
    print("Getting robot state... ")
    self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
    self._init_state = self._rs.state().enabled
    print("Enabling robot... ")
    self._rs.enable()
    
    self.server_pid_path = None

    # moveit_commander.roscpp_initialize(sys.argv)
    # self.robot_commander = moveit_commander.RobotCommander()
    # self.left_commander = moveit_commander.MoveGroupCommander('left_arm')
    # self.right_commander = moveit_commander.MoveGroupCommander('right_arm')
    # self.display_trajectory_publisher = rospy.Publisher(
    #                                   '/move_group/display_planned_path',
    #                                   moveit_msgs.msg.DisplayTrajectory)
    # print("Sleeping to allow RVIZ to start up")
    # rospy.sleep(10)
    # import pdb; pdb.set_trace()




  '''
  Print information for the end effector links
  Print out the current state of the robot
  '''
  def printRobotInfo(self):
    print("============ Left End effector Link: ", self.left_commander.get_end_effector_link())
    print("============ Right End effector Link: ", self.right_commander.get_end_effector_link())
    print("============ Robot state: ", self.robot_commander.get_current_state())
  
  def getEndEffectorLinkName(self, limb='left'):
    if limb == 'left':
      return self.left_commander.get_end_effector_link()
    return self.right_commander.get_end_effector_link()

  # Shut down MoveIt!
  def shutdown(self):
    moveit_commander.roscpp_shutdown()

  def gripperOpen(self, limb='left'):
    if limb == 'left':
      self._left_gripper.open()
    else:
      self._right_gripper.open()
    rospy.sleep(1.0)

  def gripperClose(self, limb='left'):
    if limb == 'left':
      self._left_gripper.close()
    else:
      self._right_gripper.close()
    rospy.sleep(1.0)

  # Move arm to position where it's ready to manipulate objects
  def moveToStart(self, start_angles=None, limb='left'):
    print("Moving the %s arm to start pose..." % limb)
    starting_angles = {'left_w0': 0.6699952259595108,
                       'left_w1': 1.030009435085784,
                       'left_w2': -0.4999997247485215,
                       'left_e0': -1.189968899785275,
                       'left_e1': 1.9400238130755056,
                       'left_s0': -0.08000397926829805,
                       'left_s1': -0.9999781166910306}
    if not start_angles and limb == 'left':
      joints = self._left_limb.joint_names()
      start_angles = dict(zip(joints, [starting_angles[joint] for joint in joints]))
      self._left_limb.move_to_joint_positions(start_angles)
      self._left_gripper.open()
    elif not start_angles:
      joints = self._right_limb.joint_names()
      start_angles = dict(zip(joints, [starting_angles[joint] for joint in joints]))
      self._right_limb.move_to_joint_positions(start_angles)
      self._right_gripper.open()

####################################################################################################
####################################### Collision Checking #########################################
####################################################################################################
  '''
  Check for collision; return False if collision, otherwise return True
  num_points is the number of points to linearly interpolate between the provided waypoints
  If no interpolation is desired, set num_points to 0
  Group can be 'left_arm' or 'right_arm'. Checks the limb for you, by default.
  Trajectory must be a list of length-7 lists.
  Each length-7 list must be a list of joint angles in the same order as self._limb.joint_names()
  '''
  def checkCollision(self, trajectory, num_points=20):
    group = self._limb
    if num_points_interpolated > 0:
      full_trajectory = []
      for waypoint_idx in xrange(len(trajectory) - 1):
        trajectory.append(waypoint)
        for idx in xrange(num_points_interpolated):
          interpolated_points = self.interpolate(
            trajectory[waypoint_idx], trajectory[waypoint_idx + 1], num_points)
          trajectory.extend(interpolated_points)
      trajectory.append(trajectory[-1])
    else:
      full_trajectory = trajectory
    for traj_point in full_trajectory:
      rs = RobotState()
      rs.joint_state.name = self._limb.joint_names()
      rs.joint_state.position = traj_point
      result = StateValidity().getStateValidity(rs, group)
      if not result.valid: # if one point is not valid, the trajectory is not valid!! :-(
        return False
    return True

  '''
  Helper method for checkCollision. 
  Returns linearly interpolated points between joint angle-space waypoints.
  '''
  def interpolate(self, start_angles, stop_angles, num_points):
    interpolated = []
    for i in xrange(len(start_angles)):
      interpolated.append(
        np.linspace(start_angles[i], end_angles[i], num=num_points, endpoint=False))
    waypoints = []
    for point_idx in xrange(num_points):
      waypoint = []
      for angle_idx in xrange(len(start_angles)):
        waypoint.append(interpolated[angle_idx][point_idx])
      waypoints.append(waypoint)
    return waypoints


####################################################################################################
################################### Trajectory Following Methods ###################################
####################################################################################################
  
  '''
  Input: trajectory is a list of lists of joint angles, where each list of joint angles has length 7
  Angles must be in the same order as self._limb.joint_names()
  '''
  def followMoveItTrajectoryWithJointAngles(self, trajectory, group='left_arm'):
    template = group.get_current_joint_values()
    for angle_position in trajectory:
      waypoint = copy.deepcopy(template)
      for i in xrange(7):
        waypoint[i] = angle_position[i]
      group.go(joint_goal, wait=True)
      group.stop()


  '''
  Follow a trajectory with consideration for time..
  Requires: joint POSITION server is running on another thread
  Requires: joint VELOCITY server is not running.
  Input: list of (angle, timestamp) or (angle, timedelta as float) pairs
  Angle can be dict or list in order: 's0, s1, e0, e1, w0, w1, w2' 
  Input ordering is the same as self._limb.joint_names()
  To run position server: rosrun baxter_interface joint_trajectory_action_server.py --mode position
  WARNING: If you give the robot way too little time to get to each position it starts behaving weirdly.
  Note: This all happens in simulated time, not real time.
  '''
  def followTrajectoryWithIK(self, trajectory, time_buffer=2., wait=True, limb='left'):
    if limb == 'left':
      joints = self._left_limb.joint_names()
    else:
      joints = self._right_limb.joint_names()
    #process joint angles
    if type(trajectory[0][0]) == dict:
      for idx, (angle, timestamp) in enumerate(trajectory):
        trajectory[idx] = ([self._limb.joint_angle(joint) for joint in joints], timestamp)
    elif type(trajectory[0][0]) != list:
      raise ValueError('Unexpected input format.')

    if type(trajectory[0][1]) in [rospy.Duration, genpy.rostime.Duration]:
      for idx, (angles, timestamp) in enumerate(trajectory):
        trajectory[idx] = (angles, timestamp.to_sec())
    elif type(trajectory[0][1]) not in [float, int]:
      raise ValueError('Unexpected input format.')

    traj = Trajectory(limb)
    rospy.on_shutdown(traj.stop)
    #command current joint positions first
    if limb == 'left':
      current_angles = [self._left_limb.joint_angle(joint) for joint in self._left_limb.joint_names()]
    else:
      current_angles = [self._right_limb.joint_angle(joint) for joint in self._right_limb.joint_names()]
    traj.add_point(current_angles, 0.0)
    #now add other positions
    for angles, delta in trajectory:
      traj.add_point(angles, delta)

    execution_time = sum([pair[1] for pair in trajectory])
    traj.start()
    if wait:
      traj.wait(execution_time + time_buffer)
    print("TRAJECTORY COMPLETED")


  '''
  Follows a joint velocity trajectory in a time-sensitive manner
  Requires: joint VELOCITY server is running on another thread
  Requires: joint POSITION server is NOT running on another thread
  Input: list of (velocities, timedelta) pairs
  Velocities must be lists in order: 's0, s1, e0, e1, w0, w1, w2' (limb_interface.joint_names())
  Timedeltas must be floats
  For velocity server: rosrun baxter_interface joint_trajectory_action_server.py --mode velocity
  '''
  def followTrajectoryFromJointVelocity(self, trajectory, stop=True, limb='left'):
    if limb == 'left':
      joint_names = self._left_limb.joint_names()
    else:
      joint_names = self._right_limb.joint_names()
    topic = 'robot/limb/%s/joint_command' % limb
    pub = rospy.Publisher(topic, JointCommand, queue_size=10)
    rate = rospy.Rate(50.)

    VELOCITY_MODE = 2
    template = JointCommand()
    template.mode = VELOCITY_MODE
    template.names.extend(joint_names)
    for pair in trajectory:
      print pair
      velocities, delta = pair
      msg = copy.deepcopy(template)
      msg.command.extend(velocities)
      pub.publish(msg)
      rospy.sleep(delta)

    if stop:
      velocities = [0.0] * len(joint_names)
      cmd = dict(zip(joint_names, velocities))
      if limb == 'left':
        self._left_limb.set_joint_velocities(cmd)
      else:
        self._right_limb.set_joint_velocities(cmd)


  '''
  Follow trajectory of joint angles, while linearly interpolating between points
  This method has NO REGARD for time
  In fact, this method has a pointed disregard for time. It's personal.
  Requires: neither joint position nor joint velocity server are running
  Input can be a list of length-7 list with angles in order: 
  's0, s1, e0, e1, w0, w1, w2' which is the same as self._limb.joint_names()
  Input can also be a list of dictionaries with joints as keys and angles as values
  '''
  def followTrajectoryFromJointAngles(self, input_trajectory, limb='left'):
    if type(input_trajectory[0]) == list:
      prefix = limb + '_'
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
      raise ValueError('Unexpected input format.')

    for joint_angles in trajectory:
      if limb=='left':
        self._left_limb.move_to_joint_positions(joint_angles)
      else:
        self._right_limb.move_to_joint_positions(joint_angles)

####################################################################################################
############################### Trajectory Following Helper Methods ################################
####################################################################################################

  '''
  Helper method.
  Input: a pose for the end effector
  Output: a joint angle dictionary
  '''
  def solveIK(self, pose, verbose=True):
    #the header content doesn't matter but you need to have one
    header = Header(stamp=rospy.Time.now(), frame_id='base')
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

  def startServer(self, limb='left', rate=100.0, mode='position', interpolation='bezier'):
    out_path = rospkg.RosPack().get_path('baxter_sim_platform') + '/scripts/pid.txt'
    command = 'rosrun baxter_interface joint_trajectory_action_server.py -l %s -r %s -m %s -i %s & echo $! > %s' % (limb, rate, mode, interpolation, out_path)
    os.system(command)
    self.server_pid_path = out_path
    rospy.sleep(3)

  def getPID(self):
    try:
      f = open(self.server_pid_path, 'r')
      return int(f.read())
    except:
      return

  def stopServer(self):
    pid = self.getPID()
    if not pid:
      return
    command = 'kill %s' % pid
    os.system(command)


####################################################################################################
###################################### Contact Sensor Methods ######################################
####################################################################################################

  #You will likely want to write more callback methods here

  def turnOnCallBacks(self):
    rospy.Subscriber("/l_gripper_l_finger_tip_contact_sensor_state", ContactsState, contactCallback)
    rospy.Subscriber("/l_gripper_r_finger_tip_contact_sensor_state", ContactsState, contactCallback)
    rospy.Subscriber("/r_gripper_l_finger_tip_contact_sensor_state", ContactsState, contactCallback)
    rospy.Subscriber("/r_gripper_r_finger_tip_contact_sensor_state", ContactsState, contactCallback)


####################################################################################################
########################################### For Testing ############################################
####################################################################################################
  '''
  Used for testing purposes only
  Translate in x, y, z by 0.2 meters each from the current position
  Then add pi/4 rotation to x, y, z, w
  Returns list of [pose, timestamp] pairs if timestamped=True
  Returns list of pose objects, otherwise
  '''
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



# You will likely want to add more custom callback methods here
def contactCallback(contact_data):
  if contact_data.states:
    print 'Contact detected!'
    print data.states