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
#include "motoman_driver/simple_message/motoman_motion_ctrl.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"
#endif

#ifdef MOTOPLUS
#include "motoman_motion_ctrl.h"
#include "shared_types.h"
#include "log_wrapper.h"
#endif

using industrial::shared_types::shared_int;
using industrial::shared_types::shared_real;

namespace motoman
{
namespace simple_message
{
namespace motion_ctrl
{
MotionCtrl::MotionCtrl(void)
{
  this->init();
}
MotionCtrl::~MotionCtrl(void)
{
}

void MotionCtrl::init()
{
  this->init(0, 0, MotionControlCmds::UNDEFINED, 0);
}

void MotionCtrl::init(shared_int robot_id, shared_int sequence,
                      MotionControlCmd command, shared_real data)
{
  this->setRobotID(robot_id);
  this->setSequence(sequence);
  this->setCommand(command);
  this->clearData();
  this->setData(0, data);
}

void MotionCtrl::copyFrom(MotionCtrl &src)
{
  this->setRobotID(src.getRobotID());
  this->setSequence(src.getSequence());
  this->setCommand(src.getCommand());
  for (size_t i = 0; i < MAX_DATA_CNT; ++i)
    this->setData(i, src.getData(i));
}

bool MotionCtrl::operator==(MotionCtrl &rhs)
{
  bool rslt = this->robot_id_ == rhs.robot_id_ &&
              this->sequence_ == rhs.sequence_ &&
              this->command_ == rhs.command_;

  for (size_t i = 0; i < MAX_DATA_CNT; ++i)
    rslt &= (this->data_[i] == rhs.data_[i]);

  return rslt;
}

bool MotionCtrl::load(industrial::byte_array::ByteArray *buffer)
{
  LOG_COMM("Executing MotionCtrl command load");

  if (!buffer->load(this->robot_id_))
  {
    LOG_ERROR("Failed to load MotionCtrl robot_id");
    return false;
  }

  if (!buffer->load(this->sequence_))
  {
    LOG_ERROR("Failed to load MotionCtrl sequence");
    return false;
  }

  if (!buffer->load(this->command_))
  {
    LOG_ERROR("Failed to load MotionCtrl command");
    return false;
  }

  for (size_t i = 0; i < MAX_DATA_CNT; ++i)
  {
    shared_real value = this->getData(i);
    if (!buffer->load(value))
    {
      LOG_ERROR("Failed to load MotionCtrl data element %d from data[%d]", static_cast<int>(i), buffer->getBufferSize());
      return false;
    }
  }

  LOG_COMM("MotionCtrl data successfully loaded");
  return true;
}

bool MotionCtrl::unload(industrial::byte_array::ByteArray *buffer)
{
  LOG_COMM("Executing MotionCtrl command unload");

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

  if (!buffer->unload(this->command_))
  {
    LOG_ERROR("Failed to unload MotionCtrl command");
    return false;
  }

  if (!buffer->unload(this->sequence_))
  {
    LOG_ERROR("Failed to unload MotionCtrl sequence");
    return false;
  }

  if (!buffer->unload(this->robot_id_))
  {
    LOG_ERROR("Failed to unload MotionCtrl robot_id");
    return false;
  }

  LOG_COMM("MotionCtrl data successfully unloaded");
  return true;
}

}  // namespace motion_ctrl
}  // namespace simple_message
}  // namespace motoman
