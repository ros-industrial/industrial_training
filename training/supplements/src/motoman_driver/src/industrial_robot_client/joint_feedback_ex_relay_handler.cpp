/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2014, Fraunhofer IPA
 * Author: Thiago de Freitas
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *  notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *  notice, this list of conditions and the following disclaimer in the
 *  documentation and/or other materials provided with the distribution.
 *  * Neither the name of the Southwest Research Institute, nor the names
 *  of its contributors may be used to endorse or promote products derived
 *  from this software without specific prior written permission.
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

#include <vector>
#include <map>
#include <algorithm>
#include <string>
#include "motoman_driver/industrial_robot_client/joint_feedback_ex_relay_handler.h"
#include "simple_message/log_wrapper.h"
#include "motoman_driver/simple_message/motoman_simple_message.h"

using industrial::joint_data::JointData;
using industrial::shared_types::shared_real;
namespace MotomanMsgTypes = motoman::simple_message::MotomanMsgTypes;
namespace ValidFieldTypes = industrial::joint_feedback::ValidFieldTypes;

namespace industrial_robot_client
{
namespace joint_feedback_ex_relay_handler
{
bool JointFeedbackExRelayHandler::init(SmplMsgConnection* connection,
                                       std::map<int, RobotGroup> &robot_groups)
{
  this->pub_joint_control_state_ =
    this->node_.advertise<control_msgs::FollowJointTrajectoryFeedback>("feedback_states", 1);
  this->dynamic_pub_joint_control_state_ =
    this->node_.advertise<motoman_msgs::DynamicJointTrajectoryFeedback>("dynamic_feedback_states", 1);

  this->pub_joint_sensor_state_ = this->node_.advertise<sensor_msgs::JointState>("joint_states", 1);

  this->robot_groups_ = robot_groups;
  this->version_0_ = false;
  bool rtn = JointRelayHandler::init(connection, static_cast<int>(MotomanMsgTypes::ROS_MSG_MOTO_JOINT_FEEDBACK_EX), robot_groups);
  // try to read groups_number parameter, if none specified
  if ((groups_number_ < 0))
    node_.param("groups_number", groups_number_, 0);
  return rtn;
}

bool JointFeedbackExRelayHandler::init(SmplMsgConnection* connection,
                                       std::vector<std::string> &joint_names)
{
  this->version_0_ = true;
  bool rtn = JointRelayHandler::init(connection, static_cast<int>(MotomanMsgTypes::ROS_MSG_MOTO_JOINT_FEEDBACK_EX), joint_names);
  // try to read groups_number parameter, if none specified
  if ((groups_number_ < 0))
    node_.param("groups_number", groups_number_, 0);
  return rtn;
}


bool JointFeedbackExRelayHandler::create_messages(SimpleMessage& msg_in,
    control_msgs::FollowJointTrajectoryFeedback* control_state,
    sensor_msgs::JointState* sensor_state)
{
  // inspect groups_number field first, to avoid "Failed to Convert" message
  JointFeedbackExMessage tmp_msg;
  tmp_msg.init(msg_in);
  motoman_msgs::DynamicJointTrajectoryFeedback dynamic_control_state;

  for (int i = 0; i < tmp_msg.getJointMessages().size(); i++)
  {
    int group_number = tmp_msg.getJointMessages()[i].getRobotID();

    create_messages(tmp_msg.getJointMessages()[i], control_state, sensor_state, group_number);
    motoman_msgs::DynamicJointState dyn_joint_state;
    dyn_joint_state.num_joints = control_state->joint_names.size();
    dyn_joint_state.group_number = group_number;
    dyn_joint_state.valid_fields = this->valid_fields_from_message_;
    dyn_joint_state.positions = control_state->actual.positions;
    dyn_joint_state.velocities = control_state->actual.velocities;
    dyn_joint_state.accelerations = control_state->actual.accelerations;
    dynamic_control_state.joint_feedbacks.push_back(dyn_joint_state);
  }
  dynamic_control_state.header.stamp = ros::Time::now();
  dynamic_control_state.num_groups = tmp_msg.getGroupsNumber();
  this->dynamic_pub_joint_control_state_.publish(dynamic_control_state);
}

bool JointFeedbackExRelayHandler::create_messages(JointFeedbackMessage& msg_in,
    control_msgs::FollowJointTrajectoryFeedback* control_state,
    sensor_msgs::JointState* sensor_state, int robot_id)
{
  DynamicJointsGroup all_joint_state;
  if (!JointFeedbackExRelayHandler::convert_message(msg_in, &all_joint_state, robot_id))
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

  this->pub_joint_control_state_.publish(*control_state);

  *sensor_state = sensor_msgs::JointState();  // always start with a "clean" message
  sensor_state->header.stamp = ros::Time::now();
  sensor_state->name = pub_joint_names;
  sensor_state->position = pub_joint_state.positions;
  sensor_state->velocity = pub_joint_state.velocities;

  this->pub_joint_sensor_state_.publish(*sensor_state);

  return true;
}


bool JointFeedbackExRelayHandler::convert_message(JointFeedbackMessage& msg_in, DynamicJointsGroup* joint_state, int robot_id)
{
  JointData values;

  int num_jnts = robot_groups_[robot_id].get_joint_names().size();

  // copy position data
  bool position_field = msg_in.getPositions(values);
  if (position_field)
  {
    this->valid_fields_from_message_ |= ValidFieldTypes::POSITION;
    if (!JointDataToVector(values, joint_state->positions, num_jnts))
    {
      LOG_ERROR("Failed to parse position data from JointFeedbackMessage");
      return false;
    }
  }
  else
  {
    joint_state->positions.clear();
    this->valid_fields_from_message_ &= ~ValidFieldTypes::POSITION;
  }

  // copy velocity data
  bool velocity_field = msg_in.getVelocities(values);
  if (velocity_field)
  {
    this->valid_fields_from_message_ |= ValidFieldTypes::VELOCITY;
    if (!JointDataToVector(values, joint_state->velocities, num_jnts))
    {
      LOG_ERROR("Failed to parse velocity data from JointFeedbackMessage");
      return false;
    }
  }
  else
  {
    joint_state->velocities.clear();
    this->valid_fields_from_message_ &= ~ValidFieldTypes::VELOCITY;
  }

  // copy acceleration data
  bool acceleration_field = msg_in.getAccelerations(values);
  if (acceleration_field)
  {
    this->valid_fields_from_message_ |= ValidFieldTypes::ACCELERATION;
    if (!JointDataToVector(values, joint_state->accelerations, num_jnts))
    {
      LOG_ERROR("Failed to parse acceleration data from JointFeedbackMessage");
      return false;
    }
  }
  else
  {
    joint_state->accelerations.clear();
    this->valid_fields_from_message_ &= ~ValidFieldTypes::ACCELERATION;
  }

  // copy timestamp data
  shared_real value;
  bool time_field = msg_in.getTime(value);
  if (time_field)
  {
    this->valid_fields_from_message_ |= ValidFieldTypes::TIME;
    joint_state->time_from_start = ros::Duration(value);
  }
  else
  {
    joint_state->time_from_start = ros::Duration(0);
    this->valid_fields_from_message_ &= ~ValidFieldTypes::TIME;
  }

  return true;
}

bool JointFeedbackExRelayHandler::JointDataToVector(const JointData &joints,
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


}  // namespace joint_feedback_ex_relay_handler
}  // namespace industrial_robot_client
