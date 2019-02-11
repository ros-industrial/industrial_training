#! /bin/bash
sudo apt update -y
sudo apt upgrade -y
sudo apt install git -y
sudo apt install meld -y
sudo apt install build-essential -y
sudo apt install libfontconfig1 -y
sudo apt install mesa-common-dev -y
sudo apt install libglu1-mesa-dev -y

cd $HOME

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt update -y
sudo apt install ros-melodic-desktop -y
sudo rosdep init
rosdep update
sudo apt install python-rosinstall -y
sudo apt install ros-melodic-perception -y
sudo apt install ros-melodic-urdf-tutorials -y
sudo apt install ros-melodic-moveit -y
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt update -y
sudo apt install python-catkin-tools -y
sudo apt install ros-melodic-openni-camera ros-melodic-openni-launch ros-melodic-openni2-camera ros-melodic-openni2-launch -y
sudo apt install pcl-tools -y
echo "source /opt/ros/melodic/setup.bash" >> $HOME/.bashrc
source $HOME/.bashrc

wget -q -O - https://dl-ssl.google.com/linux/linux_signing_key.pub | sudo apt-key add - 
sudo sh -c 'echo "deb https://dl.google.com/linux/chrome/deb/ stable main" >> /etc/apt/sources.list.d/google.list'
sudo apt-get update
sudo apt-get install google-chrome-stable
#then install clipboardy extension

# NOTE: no way (yet?) to do headless QT IFW install.  Do this last, but will require user action
QTFILE=qtcreator-ros-bionic-latest-online-installer.run
wget -q https://aeswiki.datasys.swri.edu/qtcreator_ros/downloads/installers/bionic/$QTFILE
chmod u+x ./$QTFILE
./$QTFILE
rm ./$QTFILE

