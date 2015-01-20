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

#ifndef MOTOMAN_DRIVER_SIMPLE_MESSAGE_MESSAGES_JOINT_FEEDBACK_EX_MESSAGE_H
#define MOTOMAN_DRIVER_SIMPLE_MESSAGE_MESSAGES_JOINT_FEEDBACK_EX_MESSAGE_H

#include <vector>

#ifndef FLATHEADERS
#include "simple_message/typed_message.h"
#include "simple_message/simple_message.h"
#include "simple_message/shared_types.h"
#include "motoman_driver/simple_message/joint_feedback_ex.h"
#else
#include "typed_message.h"
#include "simple_message.h"
#include "shared_types.h"
#include "joint_feedback_ex.h"
#endif

namespace industrial
{
namespace joint_feedback_ex_message
{
/**
 * \brief Class encapsulated joint feedback ex message generation methods
 * (either to or from a industrial::simple_message::SimpleMessage type.
 *
 * This message simply wraps the industrial::joint_feedback_ex_message::JointFeedbackExMessage data type.
 * The data portion of this typed message matches JointFeedbackExMessage.
 *
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class JointFeedbackExMessage : public industrial::typed_message::TypedMessage
{
public:
  /**
   * \brief Default constructor
   *
   * This method creates an empty message.
   *
   */
  JointFeedbackExMessage(void);
  /**
   * \brief Destructor
   *
   */
  ~JointFeedbackExMessage(void);
  /**
   * \brief Initializes message from a simple message
   *
   * \param simple message to construct from
   *
   * \return true if message successfully initialized, otherwise false
   */
  bool init(industrial::simple_message::SimpleMessage & msg);

  /**
   * \brief Initializes message from a joint feedback structure
   *
   * \param joint feedback data structure
   *
   */
  void init(industrial::joint_feedback_ex::JointFeedbackEx &data);

  /**
   * \brief Initializes a new message
   *
   */
  void init();

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);

  unsigned int byteLength()
  {
    return this->data_.byteLength();
  }

  industrial::shared_types::shared_int getGroupsNumber()
  {
    return this->data_.getGroupsNumber();
  }

  std::vector<industrial::joint_feedback_message::JointFeedbackMessage> getJointMessages()
  {
    return this->data_.getJointMessages();
  }

private:
  industrial::joint_feedback_ex::JointFeedbackEx data_;
};
}  // namespace joint_feedback_ex_message
}  // namespace industrial


#endif  // MOTOMAN_DRIVER_SIMPLE_MESSAGE_MESSAGES_JOINT_FEEDBACK_EX_MESSAGE_H
