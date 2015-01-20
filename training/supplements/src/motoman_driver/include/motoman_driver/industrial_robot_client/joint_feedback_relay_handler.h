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


#ifndef MOTOMAN_DRIVER_INDUSTRIAL_ROBOT_CLIENT_JOINT_FEEDBACK_RELAY_HANDLER_H
#define MOTOMAN_DRIVER_INDUSTRIAL_ROBOT_CLIENT_JOINT_FEEDBACK_RELAY_HANDLER_H

#include <map>
#include <string>
#include <vector>
#include "motoman_driver/industrial_robot_client/joint_relay_handler.h"
#include "simple_message/messages/joint_feedback_message.h"
#include "motoman_msgs/DynamicJointsGroup.h"

namespace industrial_robot_client
{
namespace joint_feedback_relay_handler
{

using industrial::joint_feedback_message::JointFeedbackMessage;
using industrial::simple_message::SimpleMessage;
using industrial::smpl_msg_connection::SmplMsgConnection;
using industrial_robot_client::joint_relay_handler::JointRelayHandler;
using trajectory_msgs::JointTrajectoryPoint;
using motoman_msgs::DynamicJointsGroup;

/**
 * \brief Message handler that relays joint positions (converts simple message
 * types to ROS message types and publishes them)
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class JointFeedbackRelayHandler : public industrial_robot_client::joint_relay_handler::JointRelayHandler
{
public:
  /**
  * \brief Constructor
  */
  JointFeedbackRelayHandler(int robot_id = -1) : robot_id_(robot_id) {};


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
  virtual bool init(SmplMsgConnection* connection,
                    std::vector<std::string> &joint_names);

  virtual bool init(SmplMsgConnection* connection,
                    std::map<int, RobotGroup> &robot_groups);

protected:
  int robot_id_;
  bool version_0_;


  /**
   * \brief Convert joint message into intermediate message-type
   *
   * \param[in] msg_in Message from robot connection
   * \param[out] joint_state JointTrajectoryPt message for intermediate processing
   */
  virtual bool convert_message(SimpleMessage& msg_in, JointTrajectoryPoint* joint_state);

  /**
   * \brief Convert joint message into intermediate message-type
   *
   * \param[in] msg_in Message from robot connection
   * \param[out] joint_state JointTrajectoryPt message for intermediate processing
   */
  virtual bool convert_message(SimpleMessage& msg_in, DynamicJointsGroup* joint_state, int robot_id);

  // override JointRelayHandler::create_messages, to check robot_id w/o error msg
  bool create_messages(SimpleMessage& msg_in,
                       control_msgs::FollowJointTrajectoryFeedback* control_state,
                       sensor_msgs::JointState* sensor_state);

  bool create_messages(SimpleMessage& msg_in,
                       control_msgs::FollowJointTrajectoryFeedback* control_state,
                       sensor_msgs::JointState* sensor_state, int robot_id);

  // Overriding some functions to get it to work now inside of the Motoman package
  virtual bool transform(const DynamicJointsGroup& state_in, DynamicJointsGroup* state_out)
  {
    *state_out = state_in;  // by default, no transform is applied
    return true;
  }

  virtual bool select(const DynamicJointsGroup& all_joint_state, const std::vector<std::string>& all_joint_names,
                      DynamicJointsGroup* pub_joint_state, std::vector<std::string>* pub_joint_names);

private:
  static bool JointDataToVector(const industrial::joint_data::JointData &joints,
                                std::vector<double> &vec, int len);

  /**
   * \brief Convert joint feedback message into intermediate message-type
   *
   * \param[in] msg_in JointFeedbackMessage from robot connection
   * \param[out] joint_state JointTrajectoryPt message for intermediate processing
   */
  bool convert_message(JointFeedbackMessage& msg_in, JointTrajectoryPoint* joint_state);

  /**
   * \brief Convert joint feedback message into intermediate message-type
   *
   * \param[in] msg_in JointFeedbackMessage from robot connection
   * \param[out] joint_state JointTrajectoryPt message for intermediate processing
   */
  bool convert_message(JointFeedbackMessage& msg_in, DynamicJointsGroup* joint_state, int robot_id);
};  // class JointFeedbackRelayHandler

}  // namespace joint_feedback_relay_handler
}  // namespace industrial_robot_cliet


#endif  // MOTOMAN_DRIVER_INDUSTRIAL_ROBOT_CLIENT_JOINT_FEEDBACK_RELAY_HANDLER_H
