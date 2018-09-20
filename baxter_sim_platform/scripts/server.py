#!/usr/bin/env python

import argparse

import rospy

from dynamic_reconfigure.server import Server

from baxter_interface.cfg import (
    PositionJointTrajectoryActionServerConfig,
    VelocityJointTrajectoryActionServerConfig,
    PositionFFJointTrajectoryActionServerConfig,
)
from joint_trajectory_action.joint_trajectory_action import (
    JointTrajectoryActionServer,
)

from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

'''
Server to run in the background and support Baxter trajectory following methods

limb input must be one of 'left', 'right', or 'both'. 
limb input determines the limbs served by the joint trajectory action server
rate input controls the trajectory control rate in Hz
mode input must be one of 'position', 'velocity', or 'position_w_id'
platform currently supports use of only position and velocity modes
interpolation input: interpolation method for trajectory generation
interpotation input must be one of 'bezier' or 'minjerk'
'''
def start_server(limb='both', rate=100.0, mode='position', interpolation='bezier'):
    if mode == 'velocity':
        dyn_cfg_srv = Server(VelocityJointTrajectoryActionServerConfig,
                             lambda config, level: config)
    elif mode == 'position':
        dyn_cfg_srv = Server(PositionJointTrajectoryActionServerConfig,
                             lambda config, level: config)
    else:
        dyn_cfg_srv = Server(PositionFFJointTrajectoryActionServerConfig,
                             lambda config, level: config)
    jtas = []
    if limb == 'both':
        jtas.append(JointTrajectoryActionServer('right', dyn_cfg_srv,
                                                rate, mode, interpolation))
        jtas.append(JointTrajectoryActionServer('left', dyn_cfg_srv,
                                                rate, mode, interpolation))
    else:
        jtas.append(JointTrajectoryActionServer(limb, dyn_cfg_srv, rate, mode, interpolation))

    def cleanup():
        for j in jtas:
            j.clean_shutdown()

    rospy.on_shutdown(cleanup)
    print("Running. Ctrl-c to quit")
    rospy.spin()
