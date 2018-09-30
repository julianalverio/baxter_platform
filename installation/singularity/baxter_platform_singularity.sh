# Set up keys for Gazebo
apt-get remove '.*gazebo.*' '.*sdformat.*' '.*ignition-math.*' '.*ignition-msgs.*' '.*ignition-transport.*' -y
sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable xenial main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
apt-get update
# Install ROS Kinetic Desktop Full Version
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list'
apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
apt-get update
apt-get install ros-kinetic-desktop-full -y
rosdep init
rosdep update
source ~/.bashrc
source /opt/ros/kinetic/setup.bash
apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential -y
# Install pip and pyassimp package
apt install python-pip -y
pip install --upgrade pip
pip install pyassimp==3.3
# Prerequisites for Baxter Platfor
apt-get install gazebo7 ros-kinetic-qt-build ros-kinetic-gazebo-ros-control ros-kinetic-gazebo-ros-pkgs ros-kinetic-ros-control ros-kinetic-control-toolbox ros-kinetic-realtime-tools ros-kinetic-ros-controllers ros-kinetic-xacro python-wstool ros-kinetic-tf-conversions ros-kinetic-kdl-parser ros-kinetic-moveit-commander -y
# Install MoveIt!
apt-get install ros-kinetic-moveit -y
cd ~/catkin_ws/src/baxter_platform/installation

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "alias resource='source ~/.bashrc'" >> ~/.bashrc
echo "alias sim_shell='cd ~/catkin_ws; ./baxter.sh sim'" >> ~/.bashrc
echo "alias sim='roslaunch baxter_gazebo baxter_world.launch'" >> ~/.bashrc
echo "alias sim_headless='roslaunch baxter_gazebo baxter_world.launch gui:=false recording:=true'" >> ~/.bashrc
echo "alias velocity_server='rosrun baxter_interface joint_trajectory_action_server.py --mode velocity'" >> ~/.bashrc
echo "alias position_server='rosrun baxter_interface joint_trajectory_action_server.py --mode position'" >> ~/.bashrc
echo "alias run='rosrun baxter_sim_platform/scripts main.py'" >> ~/.bashrc
echo "alias set_stuff='rosparam set /robot_description_semantic -t ~/catkin_ws/src/baxter_platform/moveit_robots/baxter/baxter_moveit_config/config/baxter.srdf'" >> ~/.bashrc
echo "alias load_stuff='rosparam load ~/catkin_ws/src/baxter_platform/moveit_robots/baxter/baxter_moveit_config/config/kinematics.yaml'" >> ~/.bashrc
echo "alias setup='set_stuff; load_stuff; rosrun moveit_ros_move_group move_group'" >> ~/.bashrc

cp ~/catkin_ws/src/baxter_platform/baxter/baxter.sh ~/catkin_ws/baxter.sh
chmod 777 ~/.ros
chmod -R 777 ~
