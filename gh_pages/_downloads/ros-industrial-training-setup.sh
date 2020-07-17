#! /bin/bash
sudo apt update -y
sudo apt upgrade -y
sudo apt install -y curl gnupg2 lsb-release git meld build-essential libfontconfig1 mesa-common-dev libglu1-mesa-dev

cd $HOME

# ROS1 packages source
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# ROS2 packages source
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

sudo apt update -y

# ROS1 install
sudo apt install -y ros-melodic-desktop
sudo apt install -y python-wstool python-catkin-tools
sudo apt install -y ros-melodic-perception ros-melodic-urdf-tutorial ros-melodic-moveit
sudo apt install -y ros-melodic-openni-camera ros-melodic-openni-launch ros-melodic-openni2-camera ros-melodic-openni2-launch
sudo apt install -y ros-melodic-industrial-core
sudo apt install -y pcl-tools

# ROS2 install
sudo apt install -y ros-eloquent-desktop
sudo apt install -y python3-colcon-common-extensions python3-argcomplete

# rosdep setup
sudo apt install python-rosdep
sudo rosdep init
rosdep update

# Install Qt Creator with ROS plugin
# NOTE: no way (yet?) to do headless QT IFW install.  Do this last, but will require user action
QTFILE=qtcreator-ros-bionic-latest-online-installer.run
wget -q https://qtcreator-ros.datasys.swri.edu/downloads/installers/bionic/$QTFILE
chmod u+x $QTFILE
./$QTFILE
rm $QTFILE
