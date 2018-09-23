# Build everything
source /opt/ros/kinetic/setup.bash
cd ~/catkin_ws; catkin_make
cp ~/catkin_ws/src/baxter/baxter.sh ~/catkin_ws/baxter.sh
# Make sure the baxter world is executable to avoid an error
sudo chmod u+x ~/catkin_ws/src/baxter_simulator/baxter_gazebo/worlds/baxter.world
# Add bash aliases
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "alias resource='source ~/.bashrc'" >> ~/.bashrc
echo "alias sim_shell='cd ~/catkin_ws; ./baxter.sh sim'" >> ~/.bashrc
echo "alias sim='roslaunch baxter_gazebo baxter_world.launch'" >> ~/.bashrc
echo "alias sim_headless='roslaunch baxter_gazebo baxter_world.launch gui:=false recording:=true'" >> ~/.bashrc
echo "alias velocity_server='rosrun baxter_interface joint_trajectory_action_server.py --mode velocity'" >> ~/.bashrc
echo "alias position_server='rosrun baxter_interface joint_trajectory_action_server.py --mode position'" >> ~/.bashrc
source ~/.bashrc