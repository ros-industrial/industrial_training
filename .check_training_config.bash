#!/bin/bash

TARGET_BRANCH=foxy  # industrial_training git branch
ROS_RELEASE=noetic     # ROS1 release version
ROS2_RELEASE=foxy      # ROS2 release version

#=======================================================================
# verify that PC configuration matches requirements for training class
#=======================================================================

function print_result() {
  if [ $? -eq 0 ]; then   # check command result
    echo -e "\e[00;32m[OK]\e[00m"
  else
    echo -e "\e[00;31m[FAIL]\e[00m"
  fi
}

# replace print_result() call with print_disabled() call to temporarily disable a test
function print_disabled() {
  echo -e "\e[00;30m[DISABLED]\e[00m"
}

function check_internet() {
  echo "Checking internet connection... "
  printf "  - %-30s" "google.com:"
  print_result $(ping -q -c1 google.com &> /dev/null)
  printf "  - %-30s" "training wiki:"
  print_result $(/usr/bin/wget -q -O /dev/null https://github.com/ros-industrial/industrial_training/wiki)

} #end check_internet()

function check_repo() {
  echo "Checking git repo status... "
  DIR=$(dirname "${BASH_SOURCE[0]}")
  printf "  - %-30s" "git repo exists:"
  print_result $(cd $DIR && git status &> /dev/null)
  printf "  - %-30s" "active branch:"
  ACTIVE_BRANCH=$(cd $DIR && git rev-parse --abbrev-ref HEAD)
  print_result $([ "$ACTIVE_BRANCH" == "$TARGET_BRANCH" ])

  # attempt to checkout correct branch, if needed
  if [ "$ACTIVE_BRANCH" != "$TARGET_BRANCH" ]; then
    printf "      %-28s" "attempting to fix branch:"
    print_result $(cd $DIR && git checkout -q $TARGET_BRANCH)
  fi

  printf "  - %-30s" "repo version:"
  REMOTE_URL=https://github.com/ros-industrial/industrial_training.git
  REMOTE_GIT=$(git ls-remote -q $REMOTE_URL $TARGET_BRANCH 2> /dev/null | cut -c1-6)
  LOCAL_GIT=$(cd $DIR && git rev-parse HEAD | cut -c1-6)
  print_result $([ "$REMOTE_GIT" == "$LOCAL_GIT" ])

  # attempt to update repo, if needed
  if [ "$REMOTE_GIT" != "$LOCAL_GIT" ]; then
    printf "      %-28s" "attempting to update repo:"
    print_result $(cd $DIR && git pull -q $REMOTE_URL)
    printf "      %-28s" "re-check repo version:"
    LOCAL_GIT=$(cd $DIR && git rev-parse HEAD | cut -c1-6)
    print_result $([ "$REMOTE_GIT" == "$LOCAL_GIT" ])
    [ "$REMOTE_GIT" != "$LOCAL_GIT" ] && printf "      remote: %s   local: %s\n" "$REMOTE_GIT" "$LOCAL_GIT"
  fi

} #end check_repo()

function check_deb() {
  printf "  - %-30s" "$1:"
  print_result $(dpkg-query -s $1 &> /dev/null)
}

function disable_deb() {
  printf "  - %-30s" "$1:"
  print_disabled $(dpkg-query -s $1 &> /dev/null)
}

function check_debs() {
  echo "Checking debian packages... "
  check_deb git
  check_deb meld
  check_deb build-essential
  check_deb libfontconfig1
  check_deb mesa-common-dev
  check_deb libglu1-mesa-dev
  check_deb pcl-tools
  check_deb python3-argcomplete
  check_deb python3-vcstool
   echo "Checking ROS1 packages:"
  check_deb python3-catkin-tools
  check_deb ros-$ROS_RELEASE-desktop
  check_deb ros-$ROS_RELEASE-perception
  check_deb ros-$ROS_RELEASE-urdf-tutorial
  check_deb ros-$ROS_RELEASE-moveit
  check_deb ros-$ROS_RELEASE-industrial-core
  disable_deb ros-$ROS_RELEASE-openni-launch
  disable_deb ros-$ROS_RELEASE-openni-camera
  disable_deb ros-$ROS_RELEASE-openni2-launch
  disable_deb ros-$ROS_RELEASE-openni2-launch
  echo "Checking ROS2 packages:"
  check_deb python3-colcon-bash
  check_deb python3-colcon-core
  check_deb python3-colcon-ros
  check_deb python3-colcon-common-extensions
  check_deb ros-$ROS2_RELEASE-desktop
  check_deb ros-$ROS2_RELEASE-moveit
  check_deb ros-$ROS2_RELEASE-ros1-bridge
  check_deb ros-$ROS2_RELEASE-ros2-control
  check_deb ros-$ROS2_RELEASE-ros2-controllers
  check_deb ros-$ROS2_RELEASE-xacro
  check_deb ros-$ROS2_RELEASE-joint-state-publisher-gui
}

function check_bashrc() {
  echo "Checking .bashrc... "
  printf "  - %-30s" "\$ROS_VERSION:"
  if [ -z ${ROS_VERSION+x} ]; then
	print_disabled $(false)
  else
	print_result $([ $ROS_VERSION == "2" ])
  fi
  printf "  - %-30s" "\$ROS_DISTRO:"
  if [ -z ${ROS_DISTRO+x} ]; then
	print_disabled $(false)
  else
	print_result $([ $ROS_DISTRO == "$ROS2_RELEASE" ])
  fi
}

function check_qtc() {
  echo "Checking for QTCreator w/ ROS plugin... "
  printf "  - %-30s" "qtcreator-ros:"
  print_result $( [ `which qtcreator-ros` ] || [ -f "$HOME/QtCreator/latest/bin/qtcreator-ros" ] )
}


#---------------------------------------
# run the actual tests
#---------------------------------------

check_internet
check_repo
check_debs
check_qtc
check_bashrc
