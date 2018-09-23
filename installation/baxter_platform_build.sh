# Build everything
source /opt/ros/kinetic/setup.bash
cd ~/catkin_ws; catkin_make
cp ~/catkin_ws/src/baxter_platform/baxter/baxter.sh ~/catkin_ws/baxter.sh
# Make sure the baxter world is executable to avoid an error
sudo chmod u+x ~/catkin_ws/src/baxter_platform/baxter_simulator/baxter_gazebo/worlds/baxter.world
# Add bash aliases
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "alias resource='source ~/.bashrc'" >> ~/.bashrc
echo "alias sim_shell='cd ~/catkin_ws; ./baxter.sh sim'" >> ~/.bashrc
echo "alias sim='roslaunch baxter_gazebo baxter_world.launch'" >> ~/.bashrc
echo "alias sim_headless='roslaunch baxter_gazebo baxter_world.launch gui:=false recording:=true'" >> ~/.bashrc
echo "alias velocity_server='rosrun baxter_interface joint_trajectory_action_server.py --mode velocity'" >> ~/.bashrc
echo "alias position_server='rosrun baxter_interface joint_trajectory_action_server.py --mode position'" >> ~/.bashrc
echo "alias run='cd ~/catkin_ws/src/baxter_platform/baxter_sim_platform/scripts; python main.py'" >> ~/.bashrc
echo "alias contact='rostopic echo /l_side_l_finger_contact_sensor_state'" >> ~/.bashrc
echo "alias camera='rosrun image_view image_view image:=/cameras/external_camera/image'" >> ~/.bashrc
echo "alias set_stuff='rosparam set /robot_description_semantic -t ~/catkin_ws/src/baxter_platform/moveit_robots/baxter/baxter_moveit_config/config/baxter.srdf'" >> ~/.bashrc
echo "alias load_stuff='rosparam load ~/catkin_ws/src/baxter_platform/moveit_robots/baxter/baxter_moveit_config/config/kinematics.yaml'" >> ~/.bashrc
echo "alias setup='set_stuff; load_stuff; rosrun baxter_interface joint_trajectory_action_server.py'" >> ~/.bashrc
source ~/.bashrc
