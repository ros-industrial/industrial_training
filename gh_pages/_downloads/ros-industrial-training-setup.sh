#! /bin/bash

# auto-detect if we're running in AWS.  Set IS_AWS before calling this script to override (IS_AWS=1 script.bash)
IS_AWS=${IS_AWS:-"$(expr "`head -c 3 /sys/hypervisor/uuid 2>/dev/null`" == "ec2" )" }

sudo apt update -y
sudo apt upgrade -y
sudo apt install -y curl gcc make gnupg2 lsb-release git meld build-essential libfontconfig1 mesa-common-dev libglu1-mesa-dev

cd $HOME

# ROS1 packages source
# sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# ROS2 packages source
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update -y

# ROS2 install
sudo apt install -y ros-humble-desktop ros-humble-moveit
sudo apt install -y ros-humble-ros1-bridge ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-xacro ros-humble-joint-state-publisher-gui
sudo apt install -y python3-colcon-common-extensions python3-argcomplete
sudo apt install -y ros-humble-pcl-ros
sudo apt install -y pcl-tools

# rosdep setup
sudo apt install python3-rosdep
sudo rosdep init
rosdep update

# Install Qt Creator with ROS plugin
# NOTE: no way (yet?) to do headless QT IFW install.  Do this last, but will require user action
if [[ $DISPLAY && ! -d ~/QtCreator ]]; then
  QTFILE=qtcreator-ros-bionic-latest-offline-installer.run
  wget -q --no-check-certificate https://qtcreator-ros.datasys.swri.edu/downloads/installers/bionic/$QTFILE
  chmod u+x $QTFILE
  ./$QTFILE
  rm $QTFILE
fi

# disable screen power-off timer
gsettings set org.gnome.desktop.session idle-delay 0

if [ $IS_AWS -eq 1 ]; then
  # setup firefox shortcuts
  xdg-icon-resource install --novendor --context apps --size 256 ~/industrial_training/gh_pages/_downloads/web_shortcuts/ros-i.png
  xdg-icon-resource install --novendor --context apps --size 128 ~/industrial_training/gh_pages/_downloads/web_shortcuts/rosorg.png
  xdg-icon-resource install --novendor --context apps --size 128 ~/industrial_training/gh_pages/_downloads/web_shortcuts/ros2.png

  sudo desktop-file-install ~/industrial_training/gh_pages/_downloads/web_shortcuts/ros-i.desktop
  sudo desktop-file-install ~/industrial_training/gh_pages/_downloads/web_shortcuts/rosorg.desktop
  sudo desktop-file-install ~/industrial_training/gh_pages/_downloads/web_shortcuts/ros2.desktop

  sudo apt install -y gnome-terminal gedit
  gsettings set org.gnome.shell favorite-apps "['firefox.desktop', 'ros-i.desktop', 'ros2.desktop', 'rosorg.desktop', 'org.gnome.Nautilus.desktop', 'org.gnome.Terminal.desktop', 'org.gnome.gedit.desktop', 'QtProject-qtcreator-ros-latest.desktop']"
  gsettings set org.gnome.desktop.wm.preferences button-layout ":minimize,maximize,close"

  # replace PS1 prompt var with "ROS Distro" prompt
  sed -E -i 's/^(\s*)(PS1=.*cloud9_prompt_user.*)$/\1#\2\n\1PS1=\'\''\\[\\e[01;32m\\]ROS$ROS_VERSION(\\[\\e[00;02;37m\\]${ROS_DISTRO:-\\[\\e[00;31m\\]NONE}\\[\\e[00;01;32m\\])\\[\\e[00m\\]:\\[\\e[01;34m\\]\\w\\[\\e[00m\\]\$ '\''/' ~/.bashrc

  # disable terminal auto-sourcing
  sed -E -i 's/^([^#].*source \/opt\/ros\/.*\/setup\..*)$/#\1/' ~/.bashrc

  # enable bash auto-completion
  echo "[[ -e /etc/profile.d/bash_completion.sh ]] && source /etc/profile.d/bash_completion.sh" >> ~/.bashrc
fi
