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

rospy.init_node("baxter_platform_manager")
rospy.wait_for_message("/robot/sim/started", Empty)
moveit_commander.roscpp_initialize(sys.argv)
self.robot_commander = moveit_commander.RobotCommander()
self.limb_commander = moveit_commander.MoveGroupCommander(self._limb_name + "_arm")
self.display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory)
print("Sleeping to allow RVIZ to start up")
rospy.sleep(10)
import pdb; pdb.set_trace()
