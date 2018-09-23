# Install Terminator
sudo apt-get install terminator -y
#Install Sublime
wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | sudo apt-key add -
echo "deb https://download.sublimetext.com/ apt/stable/" | sudo tee /etc/apt/sources.list.d/sublime-text.list
sudo apt-get update
sudo apt-get install sublime-text -y
# Set up keys for Gazebo
sudo apt-get remove '.*gazebo.*' '.*sdformat.*' '.*ignition-math.*' '.*ignition-msgs.*' '.*ignition-transport.*' -y
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
# Install ROS Kinetic Desktop Full Version
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full -y
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
source /opt/ros/kinetic/setup.bash
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential -y
mkdir ~/catkin_ws/src -p;
# Install pip and pyassimp package
sudo apt install python-pip -y
pip install --upgrade pip
sudo -H pip install pyassimp==3.3
# Prerequisites for Baxter Platform
sudo apt-get install gazebo7 ros-kinetic-qt-build ros-kinetic-gazebo-ros-control ros-kinetic-gazebo-ros-pkgs ros-kinetic-ros-control ros-kinetic-control-toolbox ros-kinetic-realtime-tools ros-kinetic-ros-controllers ros-kinetic-xacro python-wstool ros-kinetic-tf-conversions ros-kinetic-kdl-parser ros-kinetic-moveit-commander -y
# Install MoveIt!
sudo apt-get install ros-kinetic-moveit -y
source /opt/ros/kinetic/setup.bash
# Move the installation folder out of the way
mv ~/catkin_ws/src/baxter_platform/installation ~/installation
# Delete the custom platform to avoid name collisions
sudo rm -rf ~/catkin_ws/src/baxter_platform 

# Install Rethink Default Platform
cd ~/catkin_ws/src
wstool init .
wstool merge https://raw.githubusercontent.com/RethinkRobotics/baxter_simulator/kinetic-devel/baxter_simulator.rosinstall
wstool update

# Install MoveIt! for Baxter
cd ~/catkin_ws/src
git clone https://github.com/ros-planning/moveit_robots.git


# Build everything
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
