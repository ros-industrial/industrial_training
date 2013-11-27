/*
* Software License Agreement (BSD License) 
*
* Copyright (c) 2013, Southwest Research Institute
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 	* Redistributions of source code must retain the above copyright
* 	notice, this list of conditions and the following disclaimer.
* 	* Redistributions in binary form must reproduce the above copyright
* 	notice, this list of conditions and the following disclaimer in the
* 	documentation and/or other materials provided with the distribution.
* 	* Neither the name of the Southwest Research Institute, nor the names 
*	of its contributors may be used to endorse or promote products derived
*	from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/ 

#include "fs100/industrial_robot_client/joint_feedback_relay_handler.h"
#include "simple_message/log_wrapper.h"

using industrial::joint_data::JointData;
using industrial::shared_types::shared_real;
using namespace industrial::simple_message;

namespace industrial_robot_client
{
namespace joint_feedback_relay_handler
{

bool JointFeedbackRelayHandler::init(SmplMsgConnection* connection,
                                     std::vector<std::string> &joint_names)
{
  bool rtn = JointRelayHandler::init(connection, (int)StandardMsgTypes::JOINT_FEEDBACK, joint_names);

  // try to read robot_id parameter, if none specified
  if ( (robot_id_ < 0) )
    node_.param("robot_id", robot_id_, 0);

  return rtn;
}


bool JointFeedbackRelayHandler::create_messages(SimpleMessage& msg_in,
                                                control_msgs::FollowJointTrajectoryFeedback* control_state,
                                                sensor_msgs::JointState* sensor_state)
{
  // inspect robot_id field first, to avoid "Failed to Convert" message
  JointFeedbackMessage tmp_msg;
  if (tmp_msg.init(msg_in) && (tmp_msg.getRobotID() != robot_id_))
  {
    LOG_COMM("Ignoring Message: robotID (%d) doesn't match expected (%d)",
             tmp_msg.getRobotID(), robot_id_);
    return false;
  }

  return JointRelayHandler::create_messages(msg_in, control_state, sensor_state);
}

bool JointFeedbackRelayHandler::convert_message(SimpleMessage& msg_in, JointTrajectoryPoint* joint_state)
{
  JointFeedbackMessage joint_feedback_msg;

  if (!joint_feedback_msg.init(msg_in))
  {
    LOG_ERROR("Failed to initialize joint feedback message");
    return false;
  }

  return convert_message(joint_feedback_msg, joint_state);
}

bool JointFeedbackRelayHandler::JointDataToVector(const JointData &joints,
                                                  std::vector<double> &vec,
                                                  int len)
{
  if ( (len<0) || (len>joints.getMaxNumJoints()) )
  {
    LOG_ERROR("Failed to copy JointData.  Len (%d) out of range (0 to %d)",
              len, joints.getMaxNumJoints());
    return false;
  }

  vec.resize(len);
  for (int i=0; i<len; ++i)
    vec[i] = joints.getJoint(i);

  return true;
}

bool JointFeedbackRelayHandler::convert_message(JointFeedbackMessage& msg_in, JointTrajectoryPoint* joint_state)
{
  JointData values;
  int num_jnts = all_joint_names_.size();

  // copy position data
  if (msg_in.getPositions(values))
  {
    if (!JointDataToVector(values, joint_state->positions, num_jnts))
    {
      LOG_ERROR("Failed to parse position data from JointFeedbackMessage");
      return false;
    }
  } else
    joint_state->positions.clear();

  // copy velocity data
  if (msg_in.getVelocities(values))
  {
    if (!JointDataToVector(values, joint_state->velocities, num_jnts))
    {
      LOG_ERROR("Failed to parse velocity data from JointFeedbackMessage");
      return false;
    }
  } else
    joint_state->velocities.clear();

  // copy acceleration data
  if (msg_in.getAccelerations(values))
  {
    if (!JointDataToVector(values, joint_state->accelerations, num_jnts))
    {
      LOG_ERROR("Failed to parse acceleration data from JointFeedbackMessage");
      return false;
    }
  } else
    joint_state->accelerations.clear();

  // copy timestamp data
  shared_real value;
  if (msg_in.getTime(value))
    joint_state->time_from_start = ros::Duration(value);
  else
    joint_state->time_from_start = ros::Duration(0);

  return true;
}

}//namespace joint_feedback_relay_handler
}//namespace industrial_robot_client




