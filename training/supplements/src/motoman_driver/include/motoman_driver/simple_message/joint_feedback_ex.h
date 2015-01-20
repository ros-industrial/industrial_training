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

#ifndef MOTOMAN_DRIVER_SIMPLE_MESSAGE_JOINT_FEEDBACK_EX_H
#define MOTOMAN_DRIVER_SIMPLE_MESSAGE_JOINT_FEEDBACK_EX_H

#ifndef FLATHEADERS
#include "simple_message/joint_data.h"
#include "simple_message/simple_message.h"
#include "simple_message/simple_serialize.h"
#include "simple_message/shared_types.h"
#include "simple_message/joint_feedback.h"
#include "simple_message/messages/joint_feedback_message.h"
#else
#include "joint_data.h"
#include "simple_message.h"
#include "simple_serialize.h"
#include "shared_types.h"
#include "joint_feedback.h"
#include "messages/joint_feedback_message.h"
#endif

#include<vector>

namespace industrial
{
namespace joint_feedback_ex
{

class JointFeedbackEx : public industrial::simple_serialize::SimpleSerialize
{
public:
  /**
   * \brief Default constructor
   *
   * This method creates empty data.
   *
   */
  JointFeedbackEx(void);
  /**
   * \brief Destructor
   *
   */
  ~JointFeedbackEx(void);

  /**
   * \brief Initializes a empty joint feedback ex
   *
   */
  void init();

  /**
   * \brief Initializes a complete joint feedback ex
   *
   */
  void init(industrial::shared_types::shared_int groups_number,
            std::vector<industrial::joint_feedback_message::JointFeedbackMessage> joints_feedback_points);

  /**
   * \brief Sets groups_number_
   *        Numbers of group, this sets the amount of control groups connected to the controller
   *
   * \param groups_number new groups_number value
   */
  void setGroupsNumber(industrial::shared_types::shared_int groups_number)
  {
    this->groups_number_ = groups_number;
  }

  void setJointMessages(std::vector<industrial::joint_feedback_message::JointFeedbackMessage> joint_feedback_messages)
  {
    this->joint_feedback_messages_ = joint_feedback_messages;
  }

  std::vector<industrial::joint_feedback_message::JointFeedbackMessage> getJointMessages()
  {
    return joint_feedback_messages_;
  }

  /**
   * \brief Gets groups_number
   *        Gets the number of groups currently running on the controller
   *
   * @return groups_number value
   */
  industrial::shared_types::shared_int getGroupsNumber()
  {
    return this->groups_number_;
  }

  /**
   * \brief Copies the passed in value
   *
   * \param src (value to copy)
   */
  void copyFrom(JointFeedbackEx &src);

  /**
   * \brief == operator implementation
   *
   * \return true if equal
   */
  bool operator==(JointFeedbackEx &rhs);

  /**
   * \brief check the validity state for a given field
   * @param field field to check
   * @return true if specified field contains valid data
   */

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);
  unsigned int byteLength()
  {
    return sizeof(industrial::shared_types::shared_int) + MAX_NUM_GROUPS * (2 * sizeof(industrial::shared_types::shared_int) + sizeof(industrial::shared_types::shared_real)
           + 3 * (this->positions_.byteLength()));
  }

private:
  /**
   * \brief Number of groups attached to the controller
   */
  industrial::shared_types::shared_int groups_number_;

  std::vector<industrial::joint_feedback_message::JointFeedbackMessage> joint_feedback_messages_;

  industrial::joint_data::JointData positions_;

  static const industrial::shared_types::shared_int MAX_NUM_GROUPS = 4;
};
}  // namespace joint_feedback_ex
}  // namespace industrial


#endif  // MOTOMAN_DRIVER_SIMPLE_MESSAGE_JOINT_FEEDBACK_EX_H
