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
#include <string>
#ifdef ROS
#include "motoman_driver/simple_message/motoman_motion_ctrl.h"
#include "motoman_driver/simple_message/motoman_motion_reply.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"
#endif

#ifdef MOTOPLUS
#include "motoman_motion_ctrl.h"
#include "motoman_motion_reply.h"
#include "shared_types.h"
#include "log_wrapper.h"
#endif

using industrial::shared_types::shared_int;
using industrial::shared_types::shared_real;
namespace MotionControlCmds = motoman::simple_message::motion_ctrl::MotionControlCmds;

namespace motoman
{
namespace simple_message
{
namespace motion_reply
{

MotionReply::MotionReply(void)
{
  this->init();
}
MotionReply::~MotionReply(void)
{
}

void MotionReply::init()
{
  this->init(0, 0, MotionControlCmds::UNDEFINED, MotionReplyResults::SUCCESS, 0, 0);
}

void MotionReply::init(shared_int robot_id, shared_int sequence,
                       shared_int command, MotionReplyResult result,
                       shared_int subcode, shared_real data)
{
  this->setRobotID(robot_id);
  this->setSequence(sequence);
  this->setCommand(command);
  this->setResult(result);
  this->setSubcode(subcode);
  this->clearData();
  this->setData(0, data);
}

std::string MotionReply::getResultString(shared_int code)
{
  switch (code)
  {
  case MotionReplyResults::SUCCESS:
    return "Success";
  case MotionReplyResults::BUSY:
    return "Busy";
  case MotionReplyResults::FAILURE:
    return "Failed";
  case MotionReplyResults::INVALID:
    return "Invalid message";
  case MotionReplyResults::ALARM:
    return "Controller alarm";
  case MotionReplyResults::NOT_READY:
    return "Not Ready";
  case MotionReplyResults::MP_FAILURE:
    return "MotoPlus Error";
  default:
    return "Unknown";
  }
}

std::string MotionReply::getSubcodeString(shared_int code)
{
  switch (code)
  {
  case MotionReplySubcodes::Invalid::UNSPECIFIED:
    return "Unknown";
  case MotionReplySubcodes::Invalid::MSGSIZE:
    return "Invalid message size";
  case MotionReplySubcodes::Invalid::MSGHEADER:
    return "Invalid header";
  case MotionReplySubcodes::Invalid::MSGTYPE:
    return "Invalid message type";
  case MotionReplySubcodes::Invalid::GROUPNO:
    return "Invalid robot ID";
  case MotionReplySubcodes::Invalid::SEQUENCE:
    return "Invalid sequence ID";
  case MotionReplySubcodes::Invalid::COMMAND:
    return "Invalid command";
  case MotionReplySubcodes::Invalid::DATA:
    return "Invalid data";
  case MotionReplySubcodes::Invalid::DATA_START_POS:
    return "Trajectory start position doesn't match current robot position";
  case MotionReplySubcodes::Invalid::DATA_POSITION:
    return "Invalid position data";
  case MotionReplySubcodes::Invalid::DATA_SPEED:
    return "Invalid velocity data";
  case MotionReplySubcodes::Invalid::DATA_ACCEL:
    return "Invalid acceleration data";
  case MotionReplySubcodes::Invalid::DATA_INSUFFICIENT:
    return "Insufficient trajectory data.  Must supply valid time, pos, and velocity fields.";

  case MotionReplySubcodes::NotReady::UNSPECIFIED:
    return "Unknown";
  case MotionReplySubcodes::NotReady::ALARM:
    return "Controller alarm active";
  case MotionReplySubcodes::NotReady::ERROR:
    return "Controller error";
  case MotionReplySubcodes::NotReady::ESTOP:
    return "E-Stop active";
  case MotionReplySubcodes::NotReady::NOT_PLAY:
    return "Controller in TEACH mode";
  case MotionReplySubcodes::NotReady::NOT_REMOTE:
    return "Controller not in REMOTE mode";
  case MotionReplySubcodes::NotReady::SERVO_OFF:
    return "Unable to enable drive power";
  case MotionReplySubcodes::NotReady::HOLD:
    return "Controller in HOLD state";
  case MotionReplySubcodes::NotReady::NOT_STARTED:
    return "MotoRos not started";
  case MotionReplySubcodes::NotReady::WAITING_ROS:
    return "Waiting on ROS";

  default:
    return "Unknown";
  }
}

void MotionReply::copyFrom(MotionReply &src)
{
  this->setRobotID(src.getRobotID());
  this->setSequence(src.getSequence());
  this->setCommand(src.getCommand());
  this->setResult(src.getResult());
  this->setSubcode(src.getSubcode());
  for (size_t i = 0; i < MAX_DATA_CNT; ++i)
    this->setData(i, src.getData(i));
}

bool MotionReply::operator==(MotionReply &rhs)
{
  bool rslt = this->robot_id_ == rhs.robot_id_ &&
              this->sequence_ == rhs.sequence_ &&
              this->command_ == rhs.command_ &&
              this->result_ == rhs.result_ &&
              this->subcode_ == rhs.subcode_;

  for (size_t i = 0; i < MAX_DATA_CNT; ++i)
    rslt &= (this->data_[i] == rhs.data_[i]);

  return rslt;
}

bool MotionReply::load(industrial::byte_array::ByteArray *buffer)
{
  LOG_COMM("Executing MotionReply command load");

  if (!buffer->load(this->robot_id_))
  {
    LOG_ERROR("Failed to load MotionReply robot_id");
    return false;
  }

  if (!buffer->load(this->sequence_))
  {
    LOG_ERROR("Failed to load MotionReply sequence");
    return false;
  }

  if (!buffer->load(this->command_))
  {
    LOG_ERROR("Failed to load MotionReply command");
    return false;
  }

  if (!buffer->load(this->result_))
  {
    LOG_ERROR("Failed to load MotionReply result");
    return false;
  }

  if (!buffer->load(this->subcode_))
  {
    LOG_ERROR("Failed to load MotionReply subcode");
    return false;
  }

  for (size_t i = 0; i < MAX_DATA_CNT; ++i)
  {
    shared_real value = this->getData(i);
    if (!buffer->load(value))
    {
      LOG_ERROR("Failed to load MotionReply data element %d from data[%d]", static_cast<int>(i), buffer->getBufferSize());
      return false;
    }
  }

  LOG_COMM("MotionReply data successfully loaded");
  return true;
}

bool MotionReply::unload(industrial::byte_array::ByteArray *buffer)
{
  LOG_COMM("Executing MotionReply command unload");

  for (int i = MAX_DATA_CNT - 1; i >= 0; --i)
  {
    shared_real value;
    if (!buffer->unload(value))
    {
      LOG_ERROR("Failed to unload message data element: %d from data[%d]", static_cast<int>(i), buffer->getBufferSize());
      return false;
    }
    this->setData(i, value);
  }

  if (!buffer->unload(this->subcode_))
  {
    LOG_ERROR("Failed to unload MotionReply subcode");
    return false;
  }

  if (!buffer->unload(this->result_))
  {
    LOG_ERROR("Failed to unload MotionReply result");
    return false;
  }

  if (!buffer->unload(this->command_))
  {
    LOG_ERROR("Failed to unload MotionReply command");
    return false;
  }

  if (!buffer->unload(this->sequence_))
  {
    LOG_ERROR("Failed to unload MotionReply sequence");
    return false;
  }

  if (!buffer->unload(this->robot_id_))
  {
    LOG_ERROR("Failed to unload MotionReply robot_id");
    return false;
  }

  LOG_COMM("MotionReply data successfully unloaded");
  return true;
}

}  // namespace motion_reply
}  // namespace simple_message
}  // namespace motoman
