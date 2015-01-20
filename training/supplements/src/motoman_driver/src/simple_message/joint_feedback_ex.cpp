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

#include <vector>

#ifndef FLATHEADERS
#include "motoman_driver/simple_message/joint_feedback_ex.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"
#else
#include "joint_feedback_ex.h"
#include "shared_types.h"
#include "log_wrapper.h"
#endif

using industrial::joint_feedback_message::JointFeedbackMessage;
using industrial::joint_feedback::JointFeedback;

namespace industrial
{
namespace joint_feedback_ex
{

JointFeedbackEx::JointFeedbackEx(void)
{
  this->init();
}
JointFeedbackEx::~JointFeedbackEx(void)
{
}

void JointFeedbackEx::init()
{
  this->groups_number_ = 0;
}

void JointFeedbackEx::init(industrial::shared_types::shared_int groups_number,
                           std::vector<joint_feedback_message::JointFeedbackMessage> joints_feedback_points)
{
  this->setGroupsNumber(groups_number);
  this->joint_feedback_messages_ = joints_feedback_points;
}

void JointFeedbackEx::copyFrom(JointFeedbackEx &src)
{
  this->setGroupsNumber(src.getGroupsNumber());
  this->joint_feedback_messages_ = src.joint_feedback_messages_;
}

bool JointFeedbackEx::operator==(JointFeedbackEx &rhs)
{
  return this->groups_number_ == rhs.groups_number_;
}

bool JointFeedbackEx::load(industrial::byte_array::ByteArray *buffer)
{
  LOG_COMM("Executing joint feedback load");


  for (int i = 0; i < this->groups_number_; i++)
  {
    if (!buffer->load(this->joint_feedback_messages_[i]))
    {
      LOG_ERROR("Failed to load the Joint Feedback messages");
      return false;
    }
  }

  if (!buffer->load(this->groups_number_))
  {
    LOG_ERROR("Failed to load joint feedback groups_number");
    return false;
  }


  LOG_COMM("Joint feedback successfully loaded");
  return true;
}

bool JointFeedbackEx::unload(industrial::byte_array::ByteArray *buffer)
{
  LOG_COMM("Executing joint feedback unload");

  if (!buffer->unloadFront(this->groups_number_))
  {
    LOG_ERROR("Failed to unload joint feedback groups_number");
    return false;
  }

  for (int i = 0; i < this->groups_number_; i++)
  {
    JointFeedbackMessage tmp_msg;
    JointFeedback j_feedback;



    if (!buffer->unload(j_feedback))
    {
      LOG_ERROR("Failed to unload joint feedback groups_number");
      return false;
    }
    tmp_msg.init(j_feedback);

    this->joint_feedback_messages_.push_back(tmp_msg);
  }


  LOG_COMM("Joint feedback successfully unloaded");
  return true;
}

}  // namespace joint_feedback_ex
}  // namespace industrial


