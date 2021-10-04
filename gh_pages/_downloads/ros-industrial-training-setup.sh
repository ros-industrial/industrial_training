#! /bin/bash
sudo apt update -y
sudo apt upgrade -y
sudo apt install -y curl gnupg2 lsb-release git meld build-essential libfontconfig1 mesa-common-dev libglu1-mesa-dev

cd $HOME

# ROS1 packages source
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# ROS2 packages source
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update -y

# ROS1 install
sudo apt install -y ros-noetic-desktop
sudo apt install -y python3-wstool python3-catkin-tools
sudo apt install -y ros-noetic-perception ros-noetic-urdf-tutorial ros-noetic-moveit
sudo apt install -y ros-noetic-openni-camera ros-noetic-openni-launch ros-noetic-openni2-camera ros-noetic-openni2-launch
sudo apt install -y ros-noetic-industrial-core
sudo apt install -y pcl-tools

# ROS2 install
sudo apt install -y ros-foxy-desktop ros-foxy-moveit
sudo apt install -y python3-colcon-common-extensions python3-argcomplete

# rosdep setup
sudo apt install python3-rosdep
sudo rosdep init
rosdep update

# Install Qt Creator with ROS plugin
# NOTE: no way (yet?) to do headless QT IFW install.  Do this last, but will require user action
QTFILE=qtcreator-ros-bionic-latest-online-installer.run
wget -q https://qtcreator-ros.datasys.swri.edu/downloads/installers/bionic/$QTFILE
chmod u+x $QTFILE
./$QTFILE
rm $QTFILE
