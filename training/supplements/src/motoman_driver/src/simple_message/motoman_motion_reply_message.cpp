/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Southwest Research Institute
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

#ifdef ROS
#include "motoman_driver/simple_message/motoman_motion_reply_message.h"
#include "simple_message/byte_array.h"
#include "simple_message/log_wrapper.h"
#endif

#ifdef MOTOPLUS
#include "motoman_motion_reply_message.h"
#include "byte_array.h"
#include "log_wrapper.h"
#endif

using industrial::byte_array::ByteArray;
using industrial::simple_message::SimpleMessage;
using motoman::simple_message::motion_reply::MotionReply;

namespace motoman
{
namespace simple_message
{
namespace motion_reply_message
{

MotionReplyMessage::MotionReplyMessage(void)
{
  this->init();
}

MotionReplyMessage::~MotionReplyMessage(void)
{
}

bool MotionReplyMessage::init(SimpleMessage & msg)
{
  ByteArray data = msg.getData();
  this->init();

  if (!data.unload(this->reply_))
  {
    LOG_ERROR("Failed to unload MotionReplyMessage data");
    return false;
  }
  return true;
}

void MotionReplyMessage::init(MotionReply & reply)
{
  this->init();
  this->reply_.copyFrom(reply);
}

void MotionReplyMessage::init()
{
  this->setMessageType(MotomanMsgTypes::MOTOMAN_MOTION_REPLY);
  this->reply_.init();
}

bool MotionReplyMessage::load(ByteArray *buffer)
{
  LOG_COMM("Executing MotionReply message load");
  if (!buffer->load(this->reply_))
  {
    LOG_ERROR("Failed to load MotionReply message");
    return false;
  }

  return true;
}

bool MotionReplyMessage::unload(ByteArray *buffer)
{
  LOG_COMM("Executing MotionReply message unload");

  if (!buffer->unload(this->reply_))
  {
    LOG_ERROR("Failed to unload MotionReply message");
    return false;
  }

  return true;
}

}  // namespace motion_reply_message
}  // namespace simple_message
}  // namespace motoman

