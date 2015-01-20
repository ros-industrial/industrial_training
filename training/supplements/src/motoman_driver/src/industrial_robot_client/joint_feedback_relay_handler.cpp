/*
* Software License Agreement (BSD License)
*
* Copyright (c) 2013, Southwest Research Institute
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   * Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*   * Neither the name of the Southwest Research Institute, nor the names
* of its contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
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

#include "motoman_driver/industrial_robot_client/joint_feedback_relay_handler.h"
#include "simple_message/log_wrapper.h"
#include <map>
#include <string>
#include <vector>

using industrial::joint_data::JointData;
using industrial::shared_types::shared_real;
namespace StandardMsgTypes = industrial::simple_message::StandardMsgTypes;

namespace industrial_robot_client
{
namespace joint_feedback_relay_handler
{

bool JointFeedbackRelayHandler::init(SmplMsgConnection* connection,
                                     std::map<int, RobotGroup> &robot_groups)
{
  this->version_0_ = false;
  bool rtn = JointRelayHandler::init(connection, static_cast<int>(StandardMsgTypes::JOINT_FEEDBACK), robot_groups);
  // try to read robot_id parameter, if none specified
  if ((robot_id_ < 0))
    node_.param("robot_id", robot_id_, 0);
  return rtn;
}

bool JointFeedbackRelayHandler::init(SmplMsgConnection* connection,
                                     std::vector<std::string> &joint_names)
{
  this->version_0_ = true;
  bool rtn = JointRelayHandler::init(connection, static_cast<int>(StandardMsgTypes::JOINT_FEEDBACK), joint_names);

  // try to read robot_id parameter, if none specified
  if ((robot_id_ < 0))
    node_.param("robot_id", robot_id_, 0);

  return rtn;
}


bool JointFeedbackRelayHandler::create_messages(SimpleMessage& msg_in,
    control_msgs::FollowJointTrajectoryFeedback* control_state,
    sensor_msgs::JointState* sensor_state)
{
  // inspect robot_id field first, to avoid "Failed to Convert" message
  JointFeedbackMessage tmp_msg;

  tmp_msg.init(msg_in);

  if (this->version_0_)
    return JointRelayHandler::create_messages(msg_in, control_state, sensor_state);
  else
    return JointFeedbackRelayHandler::create_messages(msg_in, control_state, sensor_state, tmp_msg.getRobotID());
}

bool JointFeedbackRelayHandler::create_messages(SimpleMessage& msg_in,
    control_msgs::FollowJointTrajectoryFeedback* control_state,
    sensor_msgs::JointState* sensor_state, int robot_id)
{
  DynamicJointsGroup all_joint_state;
  if (!convert_message(msg_in, &all_joint_state, robot_id))
  {
    LOG_ERROR("Failed to convert SimpleMessage");
    return false;
  }
  // apply transform, if required
  DynamicJointsGroup xform_joint_state;
  if (!transform(all_joint_state, &xform_joint_state))
  {
    LOG_ERROR("Failed to transform joint state");
    return false;
  }

  // select specific joints for publishing
  DynamicJointsGroup pub_joint_state;
  std::vector<std::string> pub_joint_names;
  if (!select(xform_joint_state, robot_groups_[robot_id].get_joint_names(), &pub_joint_state, &pub_joint_names))
  {
    LOG_ERROR("Failed to select joints for publishing");
    return false;
  }
  // assign values to messages
  *control_state = control_msgs::FollowJointTrajectoryFeedback();  // always start with a "clean" message
  control_state->header.stamp = ros::Time::now();
  control_state->joint_names = pub_joint_names;
  control_state->actual.positions = pub_joint_state.positions;
  control_state->actual.velocities = pub_joint_state.velocities;
  control_state->actual.accelerations = pub_joint_state.accelerations;
  control_state->actual.time_from_start = pub_joint_state.time_from_start;

  *sensor_state = sensor_msgs::JointState();  // always start with a "clean" message
  sensor_state->header.stamp = ros::Time::now();
  sensor_state->name = pub_joint_names;
  sensor_state->position = pub_joint_state.positions;
  sensor_state->velocity = pub_joint_state.velocities;

  this->pub_controls_[robot_id].publish(*control_state);
  this->pub_states_[robot_id].publish(*sensor_state);

  return true;
}

bool JointFeedbackRelayHandler::convert_message(SimpleMessage& msg_in, DynamicJointsGroup* joint_state, int robot_id)
{
  JointFeedbackMessage joint_feedback_msg;
  if (!joint_feedback_msg.init(msg_in))
  {
    LOG_ERROR("Failed to initialize joint feedback message");
    return false;
  }

  return convert_message(joint_feedback_msg, joint_state, robot_id);
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
  if ((len < 0) || (len > joints.getMaxNumJoints()))
  {
    LOG_ERROR("Failed to copy JointData.  Len (%d) out of range (0 to %d)",
              len, joints.getMaxNumJoints());
    return false;
  }

  vec.resize(len);
  for (int i = 0; i < len; ++i)
    vec[i] = joints.getJoint(i);

  return true;
}

bool JointFeedbackRelayHandler::convert_message(JointFeedbackMessage& msg_in, DynamicJointsGroup* joint_state, int robot_id)
{
  JointData values;
  int num_jnts = robot_groups_[robot_id].get_joint_names().size();
  // copy position data
  if (msg_in.getPositions(values))
  {
    if (!JointDataToVector(values, joint_state->positions, num_jnts))
    {
      LOG_ERROR("Failed to parse position data from JointFeedbackMessage");
      return false;
    }
  }
  else
    joint_state->positions.clear();

  // copy velocity data
  if (msg_in.getVelocities(values))
  {
    if (!JointDataToVector(values, joint_state->velocities, num_jnts))
    {
      LOG_ERROR("Failed to parse velocity data from JointFeedbackMessage");
      return false;
    }
  }
  else
    joint_state->velocities.clear();

  // copy acceleration data
  if (msg_in.getAccelerations(values))
  {
    if (!JointDataToVector(values, joint_state->accelerations, num_jnts))
    {
      LOG_ERROR("Failed to parse acceleration data from JointFeedbackMessage");
      return false;
    }
  }
  else
    joint_state->accelerations.clear();

  // copy timestamp data
  shared_real value;
  if (msg_in.getTime(value))
    joint_state->time_from_start = ros::Duration(value);
  else
    joint_state->time_from_start = ros::Duration(0);

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
  }
  else
    joint_state->positions.clear();

  // copy velocity data
  if (msg_in.getVelocities(values))
  {
    if (!JointDataToVector(values, joint_state->velocities, num_jnts))
    {
      LOG_ERROR("Failed to parse velocity data from JointFeedbackMessage");
      return false;
    }
  }
  else
    joint_state->velocities.clear();

  // copy acceleration data
  if (msg_in.getAccelerations(values))
  {
    if (!JointDataToVector(values, joint_state->accelerations, num_jnts))
    {
      LOG_ERROR("Failed to parse acceleration data from JointFeedbackMessage");
      return false;
    }
  }
  else
    joint_state->accelerations.clear();

  // copy timestamp data
  shared_real value;
  if (msg_in.getTime(value))
    joint_state->time_from_start = ros::Duration(value);
  else
    joint_state->time_from_start = ros::Duration(0);

  return true;
}

bool JointFeedbackRelayHandler::select(const DynamicJointsGroup& all_joint_state, const std::vector<std::string>& all_joint_names,
                                       DynamicJointsGroup* pub_joint_state, std::vector<std::string>* pub_joint_names)
{

  ROS_ASSERT(all_joint_state.positions.size() == all_joint_names.size());

  *pub_joint_state = DynamicJointsGroup();  // start with a "clean" message
  pub_joint_names->clear();

  // skip over "blank" joint names
  for (int i = 0; i < all_joint_names.size(); ++i)
  {
    if (all_joint_names[i].empty())
      continue;
    pub_joint_names->push_back(all_joint_names[i]);
    if (!all_joint_state.positions.empty())
      pub_joint_state->positions.push_back(all_joint_state.positions[i]);
    if (!all_joint_state.velocities.empty())
      pub_joint_state->velocities.push_back(all_joint_state.velocities[i]);
    if (!all_joint_state.accelerations.empty())
      pub_joint_state->accelerations.push_back(all_joint_state.accelerations[i]);
  }
  pub_joint_state->time_from_start = all_joint_state.time_from_start;

  return true;
}

}  // namespace joint_feedback_relay_handler
}  // namespace industrial_robot_client




