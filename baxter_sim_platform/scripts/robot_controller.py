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


import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import String

from kinematics_interface import StateValidity





class RobotController(object):
  def __init__(self, limb='left'):
    self._limb_name = limb # string
    self._limb = baxter_interface.Limb(limb)
    self._gripper = baxter_interface.Gripper(limb)
    ns = "ExternalTfools/" + limb + "/PositionKinematicsNode/IKService"
    self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    rospy.wait_for_service(ns, 5.0)
    # verify robot is enabled
    print("Getting robot state... ")
    self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
    self._init_state = self._rs.state().enabled
    print("Enabling robot... ")
    self._rs.enable()
    print("robot enabled")
    self.robot_commander = moveit_commander.RobotCommander()
    self.left_commander = moveit_commander.MoveGroupCommander("left_arm")
    self.right_commander = moveit_commander.MoveGroupCommander("right_arm")
    self.display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory)
    print("Sleeping to allow RVIZ to start up")
    rospy.sleep(10)


  def jointAngleToPosition(self):
  	pass
  	#$$$$$

	def followMoveItTrajectoryWithPosition(self, poses, group='left_arm', resolution=0.01, visualize=False, check_collision=True):
		#setting the last parameter to zero skips the "jump" step
		plan, fraction = group.compute_cartesian_path(waypoints, resolution, 0.0)
		if visualize:
			display_trajectory = moveit_msgs.msg.DisplayTrajectory()
			display_trajectory.trajectory_start = self.robot_commander.get_current_state()
			display_trajectory.trajectory.append(plan)
			# Publish
			self.display_trajectory_publisher.publish(display_trajectory)
		if check_collision:
			if not checkCollision(plan, wait=True):
				group.execute(plan, wait=True)
		else:
			group.execute(plan(wait=True))


	# Input: a list of lists of joint angles, where each list of joint angles has length 7.
	# Angles must be in the same order as self._limb.joint_names()
	def followMoveItTrajectoryWithJointAngles(self, angles, group='left_arm'):
		for angle in angles:
			joint_goal = group.get_current_joint_values()
			for idx in xrange(7):
				joint_goal[idx] = angle[idx]
			group.go(joint_goal, wait=True)
			group.stop()


	# Check for collision; return False if collision, otherwise return True
	def checkCollision(self, trajectory, groups=['left_arm']):
		for traj_point in trajectory.joint_trajectory.points:
			rs = RobotState()
			rs.joint_state.name = trajectory.joint_trajectory.joint_names
			rs.joint_state.position = traj_point.positions
			for group in groups:
				result = StateValidity().getStateValidity(rs, group)
				if not result.valid: # if one point is not valid, the trajectory is not valid >:(
					return False
			return True
  
  def shutdown(self):
  	moveit_commander.roscpp_shutdown()


  def showRobotState(self):
  	print(self.robot_commander.get_current_state())


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
    self._gripper.open()

  #returns a joint angle dictionary
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


  #used for testing purposes only
  #translate in x, y, z by 0.2 meters each from the current position
  #then add pi/4 rotation to x, y, z, w
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


  #Follow a trajectory with consideration for time.
  #Requires joint position server is running on another thread
  #Requires joint velocity server is not running.
  #input: list of (angle, timestamp) or (angle, timedelta as float) pairs
  #angle can be dict or list in order: 's0, s1, e0, e1, w0, w1, w2' 
  #input ordering is the same as self._limb.joint_names()
  #for position server: rosrun baxter_interface joint_trajectory_action_server.py --mode position
  #WARNING: if you give the robot too little time to get to each position it starts behaving weirdly.
  def followTrajectoryWithIK(self, trajectory, time_buffer=2., wait=True):
    joints = self._limb.joint_names()
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

    traj = Trajectory(self._limb.name)
    rospy.on_shutdown(traj.stop)
    #command current joint positions first
    current_angles = [self._limb.joint_angle(joint) for joint in self._limb.joint_names()]
    traj.add_point(current_angles, 0.0)
    #now add other positions
    for angles, delta in trajectory:
      traj.add_point(angles, delta)

    execution_time = sum([pair[1] for pair in trajectory])
    traj.start()
    if wait:
      traj.wait(execution_time + time_buffer)
    print("TRAJECTORY COMPLETED")


  #Follows a joint velocity trajectory in a time-sensitive manner
  #requires joint velocity server to be running on another thread!
  #requires joint position server is not running
  #input: list of (velocities, timedelta) pairs
  #velocities must be lists in order: 's0, s1, e0, e1, w0, w1, w2' (limb_interface.joint_names())
  #timedeltas must be floats
  #for velocity server: rosrun baxter_interface joint_trajectory_action_server.py --mode velocity
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


  #Follow trajectory of joint angles, while linearly interpolating between points
  #No consideration for time
  #Requires neither joint position nor joint velocity server are running
  #input can be a list of length-7 list with angles in order: 's0, s1, e0, e1, w0, w1, w2' (self._limb.joint_names())
  #input can also be a list of dictionaries with joints as keys and angles as values
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
      raise ValueError('Unexpected input format.')

    for joint_angles in trajectory:
      self._limb.move_to_joint_positions(joint_angles)
