/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
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


#ifndef MOTOMAN_DRIVER_INDUSTRIAL_ROBOT_CLIENT_JOINT_RELAY_HANDLER_H
#define MOTOMAN_DRIVER_INDUSTRIAL_ROBOT_CLIENT_JOINT_RELAY_HANDLER_H

#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include "ros/ros.h"
#include "control_msgs/FollowJointTrajectoryFeedback.h"
#include "sensor_msgs/JointState.h"
#include "simple_message/message_handler.h"
#include "simple_message/messages/joint_message.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "motoman_driver/industrial_robot_client/robot_group.h"
#include "motoman_msgs/DynamicJointsGroup.h"

namespace industrial_robot_client
{
namespace joint_relay_handler
{

using industrial::joint_message::JointMessage;
using industrial::simple_message::SimpleMessage;
using trajectory_msgs::JointTrajectoryPoint;
using motoman_msgs::DynamicJointsGroup;
/**
 * \brief Message handler that relays joint positions (converts simple message
 * types to ROS message types and publishes them)
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class JointRelayHandler : public industrial::message_handler::MessageHandler
{
public:

  /**
  * \brief Constructor
  */
  JointRelayHandler() {};

  typedef std::map<int, RobotGroup>::iterator it_type;
  /**
   * \brief Class initializer
   *
   * \param connection simple message connection that will be used to send replies.
   * \param joint_names list of joint-names for msg-publishing.
   *   - Count and order should match data from robot connection.
   *   - Use blank-name to exclude a joint from publishing.
   *
   * \return true on success, false otherwise (an invalid message type)
   */
  virtual bool init(industrial::smpl_msg_connection::SmplMsgConnection* connection, std::map<int, RobotGroup> &robot_groups)
  {
    return init(connection, static_cast<int>(industrial::simple_message::StandardMsgTypes::JOINT), robot_groups);
  }

  /**
   * \brief Class initializer
   *
   * \param connection simple message connection that will be used to send replies.
   * \param joint_names list of joint-names for msg-publishing.
   *   - Count and order should match data from robot connection.
   *   - Use blank-name to exclude a joint from publishing.
   *
   * \return true on success, false otherwise (an invalid message type)
   */
  virtual bool init(industrial::smpl_msg_connection::SmplMsgConnection* connection, std::vector<std::string> &joint_names)
  {
    return init(connection, static_cast<int>(industrial::simple_message::StandardMsgTypes::JOINT), joint_names);
  }

protected:
  std::vector<std::string> all_joint_names_;
  std::map<int, RobotGroup> robot_groups_;

  ros::Publisher pub_joint_control_state_;
  ros::Publisher pub_joint_sensor_state_;
  ros::NodeHandle node_;

  std::map<int, ros::Publisher> pub_controls_;
  std::map<int, ros::Publisher> pub_states_;

  /**
   * \brief Class initializer
   *
   * \param connection simple message connection that will be used to send replies.
   * \param msg_type type of simple message processed by this handler
   * \param joint_names list of joint-names for msg-publishing.
   *   - Count and order should match data from robot connection.
   *   - Use blank-name to exclude a joint from publishing.
   *
   * \return true on success, false otherwise (an invalid message type)
   */
  virtual bool init(industrial::smpl_msg_connection::SmplMsgConnection* connection,
                    int msg_type, std::vector<std::string> &joint_names);

  /**
   * \brief Class initializer
   *
   * \param connection simple message connection that will be used to send replies.
   * \param msg_type type of simple message processed by this handler
   * \param joint_names list of joint-names for msg-publishing.
   *   - Count and order should match data from robot connection.
   *   - Use blank-name to exclude a joint from publishing.
   *
   * \return true on success, false otherwise (an invalid message type)
   */
  virtual bool init(industrial::smpl_msg_connection::SmplMsgConnection* connection,
                    int msg_type, std::map<int, RobotGroup> &robot_groups);
  /**
   * \brief Convert joint message into publish message-types
   *
   * \param[in] msg_in Message from robot connection
   * \param[out] control_state FollowJointTrajectoryFeedback message for ROS publishing
   * \param[out] sensor_state JointState message for ROS publishing
   *
   * \return true on success, false otherwise
   */
  virtual bool create_messages(SimpleMessage& msg_in,
                               control_msgs::FollowJointTrajectoryFeedback* control_state,
                               sensor_msgs::JointState* sensor_state);

  /**
   * \brief Convert joint message into publish message-types
   *
   * \param[in] msg_in Message from robot connection
   * \param[out] control_state FollowJointTrajectoryFeedback message for ROS publishing
   * \param[out] sensor_state JointState message for ROS publishing
   *
   * \return true on success, false otherwise
   */
  virtual bool create_messages(SimpleMessage& msg_in,
                               control_msgs::FollowJointTrajectoryFeedback* control_state,
                               sensor_msgs::JointState* sensor_state, int robot_id);

  /**
   * \brief Convert joint message into intermediate message-type
   *
   * \param[in] msg_in Message from robot connection
   * \param[out] joint_state JointTrajectoryPt message for intermediate processing
   */
  virtual bool convert_message(SimpleMessage& msg_in, DynamicJointsGroup* joint_state, int robot_id);

  /**
   * \brief Convert joint message into intermediate message-type
   *
   * \param[in] msg_in Message from robot connection
   * \param[out] joint_state JointTrajectoryPt message for intermediate processing
   */
  virtual bool convert_message(SimpleMessage& msg_in, JointTrajectoryPoint* joint_state);

  /**
   * \brief Transform joint state before publishing.
   * Can be overridden to implement, e.g. robot-specific joint coupling.
   *
   * \param[in] state_in joint state, exactly as passed from robot connection.
   * \param[out] state_out transformed joint state (in same order/count as input state)
   *
   * \return true on success, false otherwise
   */
  virtual bool transform(const JointTrajectoryPoint& state_in, JointTrajectoryPoint* state_out)
  {
    *state_out = state_in;  // by default, no transform is applied
    return true;
  }

  /**
   * \brief Transform joint state before publishing.
   * Can be overridden to implement, e.g. robot-specific joint coupling.
   *
   * \param[in] state_in joint state, exactly as passed from robot connection.
   * \param[out] state_out transformed joint state (in same order/count as input state)
   *
   * \return true on success, false otherwise
   */
  virtual bool transform(const DynamicJointsGroup& state_in, DynamicJointsGroup* state_out)
  {
    *state_out = state_in;  // by default, no transform is applied
    return true;
  }


  /**
   * \brief Select specific joints for publishing
   *
   * \param[in] all_joint_state joint state, in count/order matching robot connection
   * \param[in] all_joint_names joint names, matching all_joint_pos
   * \param[out] pub_joint_state joint state selected for publishing
   * \param[out] pub_joint_names joint names selected for publishing
   *
   * \return true on success, false otherwise
   */
  virtual bool select(const JointTrajectoryPoint& all_joint_state, const std::vector<std::string>& all_joint_names,
                      JointTrajectoryPoint* pub_joint_state, std::vector<std::string>* pub_joint_names);

  virtual bool select(const DynamicJointsGroup& all_joint_state, const std::vector<std::string>& all_joint_names,
                      DynamicJointsGroup* pub_joint_state, std::vector<std::string>* pub_joint_names);

  /**
   * \brief Callback executed upon receiving a joint message
   *
   * \param in incoming message
   *
   * \return true on success, false otherwise
   */
  bool internalCB(SimpleMessage& in);

private:
  /**
   * \brief Convert joint message into intermediate message-type
   *
   * \param[in] msg_in JointMessage from robot connection
   * \param[out] joint_state JointTrajectoryPt message for intermediate processing
   */
  bool convert_message(JointMessage& msg_in, JointTrajectoryPoint* joint_state);

  /**
   * \brief Convert joint message into intermediate message-type
   *
   * \param[in] msg_in JointMessage from robot connection
   * \param[out] joint_state JointTrajectoryPt message for intermediate processing
   */
  bool convert_message(JointMessage& msg_in, DynamicJointsGroup* joint_state, int robot_id);

};  // class JointRelayHandler

}  // namespace joint_relay_handler
}  // namespace industrial_robot_cliet


#endif  // MOTOMAN_DRIVER_INDUSTRIAL_ROBOT_CLIENT_JOINT_RELAY_HANDLER_H
