#ifndef MOTOMAN_DRIVER_INDUSTRIAL_ROBOT_CLIENT_JOINT_FEEDBACK_EX_RELAY_HANDLER_H
#define MOTOMAN_DRIVER_INDUSTRIAL_ROBOT_CLIENT_JOINT_FEEDBACK_EX_RELAY_HANDLER_H

#include <vector>
#include <map>
#include <string>
#include "motoman_driver/industrial_robot_client/joint_relay_handler.h"
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
 *  * Neither the name of the Fraunhofer IPA, nor the names
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

#include "motoman_driver/industrial_robot_client/joint_feedback_relay_handler.h"
#include "motoman_driver/simple_message/messages/joint_feedback_ex_message.h"
#include "motoman_msgs/DynamicJointsGroup.h"
#include "motoman_msgs/DynamicJointTrajectoryFeedback.h"

namespace industrial_robot_client
{
namespace joint_feedback_ex_relay_handler
{

using industrial::joint_feedback_ex_message::JointFeedbackExMessage;
using industrial::joint_feedback_message::JointFeedbackMessage;
using industrial::simple_message::SimpleMessage;
using industrial::smpl_msg_connection::SmplMsgConnection;
using industrial_robot_client::joint_relay_handler::JointRelayHandler;
using industrial_robot_client::joint_feedback_relay_handler::JointFeedbackRelayHandler;
using trajectory_msgs::JointTrajectoryPoint;
using motoman_msgs::DynamicJointsGroup;
using motoman_msgs::DynamicJointTrajectoryFeedback;

/**
 * \brief Message handler that relays joint positions (converts simple message
 * types to ROS message types and publishes them)
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class JointFeedbackExRelayHandler : public industrial_robot_client::joint_relay_handler::JointRelayHandler
{
public:
  /**
  * \brief Constructor
  */
  JointFeedbackExRelayHandler(int groups_number = -1) : groups_number_(groups_number) {};


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
  int groups_number_;
  bool version_0_;

  ros::Publisher pub_joint_control_state_;
  ros::Publisher dynamic_pub_joint_control_state_;
  ros::Publisher pub_joint_sensor_state_;

  /**
   * \brief Convert joint message into intermediate message-type
   *
   * \param[in] msg_in Message from robot connection
   * \param[out] joint_state JointTrajectoryPt message for intermediate processing
   */


  virtual bool convert_message(JointFeedbackMessage& msg_in, DynamicJointsGroup* joint_state, int robot_id);

  // override JointRelayHandler::create_messages, to check robot_id w/o error msg
  bool create_messages(SimpleMessage& msg_in,
                       control_msgs::FollowJointTrajectoryFeedback* control_state,
                       sensor_msgs::JointState* sensor_state);

  // override JointRelayHandler::create_messages, to check robot_id w/o error msg
  bool create_messages(JointFeedbackMessage& msg_in,
                       control_msgs::FollowJointTrajectoryFeedback* control_state,
                       sensor_msgs::JointState* sensor_state, int robot_id);

private:

  static bool JointDataToVector(const industrial::joint_data::JointData &joints,
                                std::vector<double> &vec, int len);


  /**
   * \brief bit-mask of (optional) fields that have been initialized with valid data
   * \see enum ValidFieldTypes
   */
  industrial::shared_types::shared_int valid_fields_from_message_;

  /**
   * \brief Convert joint feedback message into intermediate message-type
   *
   * \param[in] msg_in JointFeedbackMessage from robot connection
   * \param[out] joint_state JointTrajectoryPt message for intermediate processing
   */
  bool convert_message(JointFeedbackExMessage& msg_in, DynamicJointsGroup* joint_state, int robot_id);
};  // class JointFeedbackExRelayHandler

}  // namespace joint_feedback_ex_relay_handler
}  // namespace industrial_robot_cliet

#endif  // MOTOMAN_DRIVER_INDUSTRIAL_ROBOT_CLIENT_JOINT_FEEDBACK_EX_RELAY_HANDLER_H
