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
sudo pip install torch
sudo pip install torchvision
sudo pip install yagmail
# Prerequisites for Baxter Platform
sudo apt-get install gazebo7 ros-kinetic-qt-build ros-kinetic-gazebo-ros-control ros-kinetic-gazebo-ros-pkgs ros-kinetic-ros-control ros-kinetic-control-toolbox ros-kinetic-realtime-tools ros-kinetic-ros-controllers ros-kinetic-xacro python-wstool ros-kinetic-tf-conversions ros-kinetic-kdl-parser ros-kinetic-moveit-commander -y
# Install MoveIt!
sudo apt-get install ros-kinetic-moveit -y
# Install Xvfb for Image Rendering
sudo apt-get install xorg -y
sudo apt-get install xvfb -y
sudo apt-get install xfonts-100dpi xfonts-75dpi xfonts-scalable xfonts-cyrillic -y
sudo apt-get install screen


cp ~/catkin_ws/src/baxter_platform/installation/baxter_platform_bashrc.bash ~/.bashrc
cp ~/catkin_ws/src/baxter_platform/baxter/baxter.sh ~/catkin_ws/baxter.sh
sudo chmod -R 777 ~/.ros
sudo chmod -R 777 ~
