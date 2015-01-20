/*
* Software License Agreement (BSD License)
*
* Copyright (c) 2011, Southwest Research Institute
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

#include <algorithm>
#include <map>
#include <vector>
#include <string>

#include "motoman_driver/industrial_robot_client/joint_relay_handler.h"
#include "simple_message/log_wrapper.h"

using industrial::message_handler::MessageHandler;
using industrial::shared_types::shared_real;
using industrial::smpl_msg_connection::SmplMsgConnection;
namespace CommTypes = industrial::simple_message::CommTypes;
namespace ReplyTypes = industrial::simple_message::ReplyTypes;

namespace industrial_robot_client
{
namespace joint_relay_handler
{

bool JointRelayHandler::init(SmplMsgConnection* connection, int msg_type, std::map<int, RobotGroup> &robot_groups)
{
  this->robot_groups_ = robot_groups;
  for (it_type iterator = robot_groups.begin(); iterator != robot_groups.end(); iterator++)
  {
    std::string name_str, ns_str;
    int robot_id = iterator->first;
    name_str = iterator->second.get_name();
    ns_str = iterator->second.get_ns();

    this->pub_joint_control_state_ =
      this->node_.advertise<control_msgs::FollowJointTrajectoryFeedback>(ns_str + "/" + name_str + "/feedback_states", 1);

    this->pub_joint_sensor_state_ = this->node_.advertise<sensor_msgs::JointState>(ns_str + "/" + name_str + "/joint_states", 1);

    this->pub_controls_[robot_id] = this->pub_joint_control_state_;
    this->pub_states_[robot_id] = this->pub_joint_sensor_state_;
  }




  return MessageHandler::init(msg_type, connection);
}

bool JointRelayHandler::init(SmplMsgConnection* connection, int msg_type, std::vector<std::string>& joint_names)
{
  this->pub_joint_control_state_ =
    this->node_.advertise<control_msgs::FollowJointTrajectoryFeedback>("feedback_states", 1);

  this->pub_joint_sensor_state_ = this->node_.advertise<sensor_msgs::JointState>("joint_states", 1);

  // save "complete" joint-name list, preserving any blank entries for later use
  this->all_joint_names_ = joint_names;

  return MessageHandler::init(msg_type, connection);
}

/*! This is responsible for publishing the generated messages */

bool JointRelayHandler::internalCB(SimpleMessage& msg_in)
{
  control_msgs::FollowJointTrajectoryFeedback control_state;
  sensor_msgs::JointState sensor_state;
  bool rtn = true;

  if (create_messages(msg_in, &control_state, &sensor_state))
  {
    rtn = true;
  }
  else
    rtn = false;

  // Reply back to the controller if the sender requested it.
  if (CommTypes::SERVICE_REQUEST == msg_in.getMessageType())
  {
    SimpleMessage reply;
    reply.init(msg_in.getMessageType(),
               CommTypes::SERVICE_REPLY,
               rtn ? ReplyTypes::SUCCESS : ReplyTypes::FAILURE);
    this->getConnection()->sendMsg(reply);
  }

  return rtn;
}

// TODO: Add support for other message fields (effort, desired pos)
bool JointRelayHandler::create_messages(SimpleMessage& msg_in,
                                        control_msgs::FollowJointTrajectoryFeedback* control_state,
                                        sensor_msgs::JointState* sensor_state)
{
  // read state from robot message
  JointTrajectoryPoint all_joint_state;
  if (!convert_message(msg_in, &all_joint_state))
  {
    LOG_ERROR("Failed to convert SimpleMessage");
    return false;
  }
  // apply transform, if required
  JointTrajectoryPoint xform_joint_state;
  if (!transform(all_joint_state, &xform_joint_state))
  {
    LOG_ERROR("Failed to transform joint state");
    return false;
  }

  // select specific joints for publishing
  JointTrajectoryPoint pub_joint_state;
  std::vector<std::string> pub_joint_names;
  if (!select(xform_joint_state, all_joint_names_, &pub_joint_state, &pub_joint_names))
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

  this->pub_joint_control_state_.publish(*control_state);
  this->pub_joint_sensor_state_.publish(*sensor_state);

  return true;
}

// TODO: Add support for other message fields (effort, desired pos)
bool JointRelayHandler::create_messages(SimpleMessage& msg_in,
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

bool JointRelayHandler::convert_message(SimpleMessage& msg_in, JointTrajectoryPoint* joint_state)
{
  JointMessage joint_msg;

  if (!joint_msg.init(msg_in))
  {
    LOG_ERROR("Failed to initialize joint message");
    return false;
  }

  return convert_message(joint_msg, joint_state);
}

bool JointRelayHandler::convert_message(SimpleMessage& msg_in, DynamicJointsGroup* joint_state, int robot_id)
{
  JointMessage joint_msg;

  if (!joint_msg.init(msg_in))
  {
    LOG_ERROR("Failed to initialize joint message");
    return false;
  }

  return convert_message(joint_msg, joint_state, robot_id);
}

bool JointRelayHandler::convert_message(JointMessage& msg_in, JointTrajectoryPoint* joint_state)
{
  // copy position data
  int num_jnts = all_joint_names_.size();
  joint_state->positions.resize(num_jnts);
  for (int i = 0; i < num_jnts; ++i)
  {
    shared_real value;
    if (msg_in.getJoints().getJoint(i, value))
    {
      joint_state->positions[i] = value;
    }
    else
      LOG_ERROR("Failed to parse position #%d from JointMessage", i);
  }

  // these fields are not provided by JointMessage
  joint_state->velocities.clear();
  joint_state->accelerations.clear();
  joint_state->time_from_start = ros::Duration(0);

  return true;
}

bool JointRelayHandler::convert_message(JointMessage& msg_in, DynamicJointsGroup* joint_state, int robot_id)
{
  // copy position data
  int num_jnts = robot_groups_[robot_id].get_joint_names().size();
  joint_state->positions.resize(num_jnts);
  for (int i = 0; i < num_jnts; ++i)
  {
    shared_real value;
    if (msg_in.getJoints().getJoint(i, value))
    {
      joint_state->positions[i] = value;
    }
    else
      LOG_ERROR("Failed to convert message");
  }

  // these fields are not provided by JointMessage
  joint_state->velocities.clear();
  joint_state->accelerations.clear();
  joint_state->time_from_start = ros::Duration(0);

  return true;
}

bool JointRelayHandler::select(const JointTrajectoryPoint& all_joint_state, const std::vector<std::string>& all_joint_names,
                               JointTrajectoryPoint* pub_joint_state, std::vector<std::string>* pub_joint_names)
{
  ROS_ASSERT(all_joint_state.positions.size() == all_joint_names.size());

  *pub_joint_state = JointTrajectoryPoint();  // start with a "clean" message
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

bool JointRelayHandler::select(const DynamicJointsGroup& all_joint_state, const std::vector<std::string>& all_joint_names,
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

}  // namespace joint_relay_handler
}  // namespace industrial_robot_client




