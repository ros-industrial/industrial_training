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
#include "motoman_driver/simple_message/motoman_motion_ctrl_message.h"
#include "simple_message/byte_array.h"
#include "simple_message/log_wrapper.h"
#endif

#ifdef MOTOPLUS
#include "motoman_motion_ctrl_message.h"
#include "byte_array.h"
#include "log_wrapper.h"
#endif

using industrial::byte_array::ByteArray;
using industrial::simple_message::SimpleMessage;
using motoman::simple_message::motion_ctrl::MotionCtrl;

namespace motoman
{
namespace simple_message
{
namespace motion_ctrl_message
{
MotionCtrlMessage::MotionCtrlMessage(void)
{
  this->init();
}

MotionCtrlMessage::~MotionCtrlMessage(void)
{
}

bool MotionCtrlMessage::init(SimpleMessage & msg)
{
  ByteArray data = msg.getData();
  this->init();

  if (!data.unload(this->cmd_))
  {
    LOG_ERROR("Failed to unload MotionCtrl data");
    return false;
  }

  return true;
}

void MotionCtrlMessage::init(MotionCtrl & cmd)
{
  this->init();
  this->cmd_.copyFrom(cmd);
}

void MotionCtrlMessage::init()
{
  this->setMessageType(MotomanMsgTypes::MOTOMAN_MOTION_CTRL);
  this->cmd_.init();
}

bool MotionCtrlMessage::load(ByteArray *buffer)
{
  LOG_COMM("Executing MotionCtrl message load");
  if (!buffer->load(this->cmd_))
  {
    LOG_ERROR("Failed to load MotionCtrl message");
    return false;
  }

  return true;
}

bool MotionCtrlMessage::unload(ByteArray *buffer)
{
  LOG_COMM("Executing MotionCtrl message unload");

  if (!buffer->unload(this->cmd_))
  {
    LOG_ERROR("Failed to unload MotionCtrl message");
    return false;
  }

  return true;
}

}  // namespace motion_ctrl_message
}  // namespace simple_message
}  // namespace motoman

