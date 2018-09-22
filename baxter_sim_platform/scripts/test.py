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
