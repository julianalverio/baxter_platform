# Get keys for Gazebo
sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable xenial main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
# Get keys for ROS Kinetic
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list'
apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

apt-get update

# Install ROS Kinetic and other prereqs
apt-get install ros-kinetic-desktop-full -y
rosdep init
rosdep update
apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential -y

# Prerequisites for Baxter Platform
apt-get install gazebo7 ros-kinetic-qt-build ros-kinetic-gazebo-ros-control ros-kinetic-gazebo-ros-pkgs ros-kinetic-ros-control ros-kinetic-control-toolbox ros-kinetic-realtime-tools ros-kinetic-ros-controllers ros-kinetic-xacro python-wstool ros-kinetic-tf-conversions ros-kinetic-kdl-parser ros-kinetic-moveit-commander -y

# Install pip and upgrade
apt install python-pip -y
pip install --upgrade pip

# Install MoveIt!
apt-get install ros-kinetic-moveit -y

cp ~/catkin_ws/src/baxter_platform/baxter/baxter.sh ~/catkin_ws/baxter.sh
chmod 777 ~/.ros
chmod -R 777 ~

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


# pip install pyassimp==3.3
#install pyassimp, source ~/.bashrc, catkin_make



#  
# rosdep init
# rosdep update
# apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential -y

# apt-get install gazebo7 ros-kinetic-qt-build ros-kinetic-gazebo-ros-control ros-kinetic-gazebo-ros-pkgs ros-kinetic-ros-control ros-kinetic-control-toolbox ros-kinetic-realtime-tools ros-kinetic-ros-controllers ros-kinetic-xacro python-wstool ros-kinetic-tf-conversions ros-kinetic-kdl-parser ros-kinetic-moveit-commander -y

# apt install python-pip -y
# pip install --upgrade pip

# apt-get install ros-kinetic-moveit -y

# cp ~/catkin_ws/src/baxter_platform/baxter/baxter.sh ~/catkin_ws/baxter.sh
# chmod 777 ~/.ros
# chmod -R 777 ~

# pip install pyassimp==3.3

# cd ~
# source .bashrc

