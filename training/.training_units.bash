#!/bin/bash

#=======================================================================
# run this code every time the file is sourced (e.g. for each new shell)

# get training directory from this file's path
TRAINING_DIR=$(dirname "$BASH_SOURCE")

# try to set ROS_PACKAGE_PATH for current training unit
TRAINING_FILE=$TRAINING_DIR/.training_unit
if [ -f $TRAINING_FILE ]; then
    . $TRAINING_FILE
fi
#=======================================================================

function clear_training_unit {
  rm -f $TRAINING_FILE
  source ~/.bashrc
}

function set_training_unit {

if [ $# -ne 2 ]
then
  echo "Usage: $FUNCNAME UNIT SUBDIR"
  return $E_BADARGS
fi

# assume function arg is unit ID (e.g. 1.2)
local UNIT=$1    # arg 1 is unitID (e.g. 1.2)
local SUBDIR=$2  # arg 2 is subdir (e.g. work or ans)
local UNIT_DIR=$TRAINING_DIR/$SUBDIR/$UNIT

# check that directory exists
if [ ! -d $UNIT_DIR ]; then
  echo -e "\n\e[00;31m  ERROR: Directory '$UNIT_DIR' does not exist.\e[00m\n"
  return
fi

# create new package path (with training_unit and supplements)
clear_training_unit # reset ROS_PACKAGE_PATH
local TRAINING_PACKAGE_PATH=$UNIT_DIR:$TRAINING_DIR/supplements

# save path (and other code) to file for re-use in new terminals
echo "ROS_PACKAGE_PATH=$TRAINING_PACKAGE_PATH:\$ROS_PACKAGE_PATH" > $TRAINING_FILE
echo "echo -e \"\n\e[00;32m  Switching to UNIT $UNIT ($SUBDIR copy)\e[00m\n\"" >> $TRAINING_FILE
echo "PS1=\"\[\e]0;ROS-I Training Unit $UNIT ($SUBDIR)\a\]\u@\h:\w\$ \"" >> $TRAINING_FILE
echo "export LIBGL_ALWAYS_SOFTWARE=1" >> $TRAINING_FILE

source $TRAINING_FILE
cd $UNIT_DIR

}   #end set_training_unit()

function training_unit {
if [ $# -ne 1 ]; then
  echo "Usage: $FUNCNAME UNIT"
else
  set_training_unit $1 work
fi
}   #end training_unit()

function training_ref {
if [ $# -ne 1 ]; then
  echo "Usage: $FUNCNAME UNIT"
else
  set_training_unit $1 ref
fi
}   #end training_ref()

function training_orig {
if [ $# -ne 1 ]; then
  echo "Usage: $FUNCNAME UNIT"
else
  set_training_unit $1 orig
fi
}   #end training_orig()
