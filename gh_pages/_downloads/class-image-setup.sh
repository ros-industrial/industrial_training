#! /bin/bash

# Script to configure specific settings for ROS-I training VM/AWS images
#  - this script also calls ros-industrial-training-setup.sh to set up common training packages/settings

# auto-detect if we're running in AWS.  Set IS_AWS before calling this script to override (export IS_AWS=1; script.bash)
IS_AWS=${IS_AWS:-"$(expr "`hostname -d`" == "ec2.internal" )" }
if [ $IS_AWS == 1 ]; then echo "****** AWS Install detected ******"; fi

# disable prompts for kernel-reboot and service-restart during apt installs
sudo sed -i "/#\$nrconf{restart} = 'i';/s/.*/\$nrconf{restart} = 'a';/" /etc/needrestart/needrestart.conf
sudo sed -i "/^#\$nrconf{kernelhints} = -1;/s/^#//" /etc/needrestart/needrestart.conf

# call primary training setup script
CURRDIR=$(dirname "${BASH_SOURCE[0]}")
$CURRDIR/ros-industrial-training-setup.sh

# install ubuntu desktop packages and remove unused packages
sudo apt install -y ubuntu-desktop-minimal gnome-terminal gedit
sudo apt remove -y aisleriot cheese gnome-mahjongg gnome-mines gnome-sudoku libreoffice-* rhythmbox shotwell thunderbird transmission-gtk && sudo apt autoremove

# remove unused home directories
rm -rf ~/Documents ~/Music ~/Pictures ~/Public ~/Templates ~/Videos

# disable screen power-off timer
gsettings set org.gnome.desktop.session idle-delay 0

# define desktop shortcuts (and icons)
xdg-icon-resource install --novendor --context apps --size 256 ~/industrial_training/gh_pages/_downloads/web_shortcuts/ros-i.png
xdg-icon-resource install --novendor --context apps --size 128 ~/industrial_training/gh_pages/_downloads/web_shortcuts/rosorg.png
xdg-icon-resource install --novendor --context apps --size 128 ~/industrial_training/gh_pages/_downloads/web_shortcuts/ros2.png
sudo desktop-file-install ~/industrial_training/gh_pages/_downloads/web_shortcuts/ros-i.desktop
sudo desktop-file-install ~/industrial_training/gh_pages/_downloads/web_shortcuts/rosorg.desktop
sudo desktop-file-install ~/industrial_training/gh_pages/_downloads/web_shortcuts/ros2.desktop

# enable dock and set desired icons
gnome-extensions enable ubuntu-dock@ubuntu.com
gsettings set org.gnome.shell favorite-apps "['firefox_firefox.desktop', 'ros-i.desktop', 'ros2.desktop', 'rosorg.desktop', 'org.gnome.Nautilus.desktop', 'org.gnome.Terminal.desktop', 'org.gnome.gedit.desktop', 'QtProject-qtcreator-ros-latest.desktop']"
 
# set min/max/close button order to match MS Windows 
gsettings set org.gnome.desktop.wm.preferences button-layout ":minimize,maximize,close"

# replace PS1 prompt var with "ROS Distro" prompt
sed -E -i 's/^(\s*)(PS1=.*\\033.*)$/\1#\2\n\1PS1=\'\''\\[\\e[01;32m\\]ROS$ROS_VERSION(\\[\\e[00;02;37m\\]${ROS_DISTRO:-\\[\\e[00;31m\\]NONE}\\[\\e[00;01;32m\\])\\[\\e[00m\\]:\\[\\e[01;34m\\]\\w\\[\\e[00m\\]\$ '\''/' ~/.bashrc

# disable terminal auto-sourcing
sed -E -i 's/^([^#].*source \/opt\/ros\/.*\/setup\..*)$/#\1/' ~/.bashrc

# configure NICE-DCV settings for AWS remote desktop
#   - ref: https://docs.aws.amazon.com/dcv/latest/adminguide/setting-up-installing-linux-prereq.html
#   - ref: https://docs.aws.amazon.com/dcv/latest/adminguide/manage-start.html
#   - ref: https://docs.aws.amazon.com/dcv/latest/adminguide/managing-sessions-start.html
#   - ref: https://docs.aws.amazon.com/dcv/latest/adminguide/security-authentication.html
if [ $IS_AWS == 1 ]; then
  sudo sed -i "/^#WaylandEnable=false/s/^#//" /etc/gdm3/custom.conf
  sudo systemctl restart gdm3
  sudo apt install xserver-xorg-video-dummy
  sudo cp $CURRDIR/dummy.xorg.conf /etc/X11/xorg.conf.d
  sudo systemctl isolate multi-user.target
  sudo systemctl isolate graphical.target
  wget https://d1uj6qtbmh3dt5.cloudfront.net/NICE-GPG-KEY
  gpg --import NICE-GPG-KEY
  rm NICE-GPG-KEY
  wget https://d1uj6qtbmh3dt5.cloudfront.net/nice-dcv-ubuntu2204-x86_64.tgz
  tar -xvzf nice-dcv-ubuntu2204-x86_64.tgz && cd nice-dcv-*/
  sudo apt install -y ./nice-dcv-server*.deb ./nice-dcv-web-viewer*.deb
  cd .. && rm -rf nice-dcv-*
  sudo systemctl enable dcvserver
  sudo sed -i "/^#create-session = true/s/^#//" /etc/dcv/dcv.conf
  sudo sed -i '/^#owner = ""/c\owner = "ubuntu"' /etc/dcv/dcv.conf
  sudo sed -i '/^#authentication="none"/c\authentication="system"' /etc/dcv/dcv.conf
  sudo systemctl stop dcvserver
  sudo systemctl start dcvserver
fi

# Install Qt Creator with ROS plugin
# NOTE: no way (yet?) to do headless QT IFW install.  Do this last, but will require user action
if [[ $DISPLAY && ! -d ~/QtCreator ]]; then
  QTFILE=qtcreator-ros-bionic-latest-offline-installer.run
  wget -q --no-check-certificate https://qtcreator-ros.datasys.swri.edu/downloads/installers/bionic/$QTFILE
  chmod u+x $QTFILE
  ./$QTFILE
  rm $QTFILE
fi
