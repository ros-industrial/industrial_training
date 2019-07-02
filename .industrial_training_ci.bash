#!/bin/bash
#********************************************************************
# Software License Agreement (BSD License)
#
#  Copyright (c) 2016, University of Colorado, Boulder
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Univ of CO, Boulder nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#********************************************************************/

# Author: Dave Coleman <dave@dav.ee>, Robert Haschke, Levi Armstrong
# Desc: Utility functions used to make CI work better

#######################################

RED='\e[1;91m'
GREEN='\e[1;92m'
YELLOW='\e[1;93m'
BLUE='\e[1;94m'
MAGENTA='\e[1;95m'
CYAN='\e[1;96m'
NC='\e[0m' # No Color

function logError {
  printf "${RED}$1${NC}\n"
}

function logWarn {
  printf "${YELLOW}$1${NC}\n"
}

function logInfo {
  printf "${NC}$1${NC}"
}

function logHighlightGreen {
  printf "${GREEN}$1${NC}\n"
}

function logHighlightBlue {
  printf "${BLUE}$1${NC}\n"
}

function logHighlightMagenta {
  printf "${MAGENTA}$1${NC}\n"
}

function logHighlightCyan {
  printf "${CYAN}$1${NC}\n"
}

#######################################
# Start a Command with timer
#
# Arguments:
#   info: info of action being ran
#######################################
function ci_time_start {
  CI_START_TIME=$SECONDS
  local COMMAND=$@

  # Output command being executed
  logHighlightGreen "$ ${COMMAND}"
}

#######################################
# Wraps up the timer section on CI (that's started mostly by ci_time_start function).
#######################################
function ci_time_end {
  if [ -z $CI_START_TIME ]; then
      printf '[ci_time_end] var CI_START_TIME is not set. You need to call `ci_time_start` in advance.';
      return;
  fi

  local CI_ELAPSED_SECONDS=$(( $SECONDS - $CI_START_TIME ))

  if (( $CI_ELAPSED_SECONDS > 3600 )) ; then
      let "hours=CI_ELAPSED_SECONDS/3600"
      let "minutes=(CI_ELAPSED_SECONDS%3600)/60"
      let "seconds=(CI_ELAPSED_SECONDS%3600)%60"
      logHighlightCyan "Completed in $hours hour(s), $minutes minute(s) and $seconds second(s)"
  elif (( $CI_ELAPSED_SECONDS > 60 )) ; then
      let "minutes=(CI_ELAPSED_SECONDS%3600)/60"
      let "seconds=(CI_ELAPSED_SECONDS%3600)%60"
      logHighlightCyan "Completed in $minutes minute(s) and $seconds second(s)"
  else
      logHighlightCyan "Completed in $CI_ELAPSED_SECONDS seconds"
  fi

  unset CI_START_TIME
}

#######################################
# Display command in CI console with start time, end time and duration
#
# Arguments:
#   command: action to run
#######################################
function ci_run_impl() {
  local command=$@

  ci_time_start $command
  # actually run command
  eval "${command}"
  result=$?
  ci_time_end
  return $result
}

#######################################
# Same as ci_run_impl() but silent command
#
# Arguments:
#   command: action to run
#######################################
function ci_run_silent_impl() {
  local command=$@

  ci_time_start "${command} > /dev/null"
  # actually run command
  eval "${command} > /dev/null"
  result=$?
  ci_time_end
  return $result
}

#######################################
# Run a command and do timing for it
#   Return the exit status of the command
#######################################
function ci_run() {
  ci_run_impl $@ || exit $?
}

#######################################
# Same as ci_run but return 0 exit status, thus ignoring any error
#######################################
function ci_run_true() {
  ci_run_impl $@ || return 0
}

#######################################
# Same as ci_run but silent output
#######################################
function ci_run_silent() {
  ci_run_silent_impl $@ || exit $?
}

function setup()
{
  ci_run apt update -qq
  ci_run apt install python-catkin-tools -qq -y
  ci_run apt install ros-$ROS_DISTRO-moveit -qq -y
  ci_run apt install ros-$ROS_DISTRO-pcl-ros -qq -y
  ci_run apt upgrade -qq -y
}

######################################################
## Build a catkin workspace with rosdep and wstool ##
######################################################
function build_ws()
{
  ci_run source /opt/ros/$ROS_DISTRO/setup.bash
  ci_run cd $HOME

  # Make a new catkin ws
  ci_run mkdir catkin_ws
  ci_run cd catkin_ws

  # Copy the exercise workspace into the home directory
  ci_run cp -r $TRAVIS_BUILD_DIR/$1 .

  # Initialize the workspace
  ci_run catkin init

  # Perform rosinstall
  ci_run wstool init
  ci_run wstool merge -t src/ src/.rosinstall
  ci_run wstool update -t src

  # Install dependencies with rosdep
  ci_run rosdep -q install --from-paths src --ignore-src -r -y

  # Build the workspace
  ci_run catkin build --no-status --summarize
}

################################################################################
## Update the container image to save time later when installing dependencies ##
################################################################################
function update_image()
{
  # Run the docker container
  ci_run docker run --name $CONTAINER_NAME --network=host -d -i \
    -v $TRAVIS_BUILD_DIR:$TRAVIS_BUILD_DIR \
    -e TRAVIS_BUILD_DIR=$TRAVIS_BUILD_DIR \
    $CONTAINER_IMAGE

  # Source this file in the docker container and run the setup routine
  ci_run docker exec -t $CONTAINER_NAME \
    bash -c \"source $TRAVIS_BUILD_DIR/.industrial_training_ci.bash\; ci_run setup\"

  # Create a new name for the updated image
  ci_run export NEW_CONTAINER_IMAGE=$CONTAINER_IMAGE\_update

  # Commit the changes to the docker and close the container
  ci_run docker commit -m \"docker setup\" $CONTAINER_NAME $NEW_CONTAINER_IMAGE
  ci_run docker stop $CONTAINER_NAME
}

#########################
## Build the exercises ##
#########################
function build_exercises()
{
  ci_run update_image

  # Create a counter for naming containers
  N=1

  for EXERCISE_SRC in "$@"
  do

  # Update the container name
  NEW_CONTAINER_NAME=$CONTAINER_NAME\_$N

  # Run the updated docker image in a new container
  ci_run docker run --name $NEW_CONTAINER_NAME --network=host -d -i \
    -v $TRAVIS_BUILD_DIR:$TRAVIS_BUILD_DIR \
    -e TRAVIS_BUILD_DIR=$TRAVIS_BUILD_DIR \
    $NEW_CONTAINER_IMAGE

  ci_run docker exec -t $NEW_CONTAINER_NAME \
    bash -c \"source $TRAVIS_BUILD_DIR/.industrial_training_ci.bash\; ci_run build_ws $EXERCISE_SRC\"

  # Close the container
  ci_run docker stop $NEW_CONTAINER_NAME

  # Increment the counter
  N=$((N+1))

  logHighlightGreen "Successfully built $EXERCISE_SRC\n"

  done

  logHighlightGreen "Successfully built all exercises\n"
}
