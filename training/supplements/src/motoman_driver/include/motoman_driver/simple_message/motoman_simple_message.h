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

#ifndef MOTOMAN_DRIVER_SIMPLE_MESSAGE_MOTOMAN_SIMPLE_MESSAGE_H
#define MOTOMAN_DRIVER_SIMPLE_MESSAGE_MOTOMAN_SIMPLE_MESSAGE_H

#ifdef ROS
#include "simple_message/simple_message.h"
#endif

#ifdef MOTOPLUS
#include "simple_message.h"
#endif


namespace motoman
{
namespace simple_message
{
/**
 * \brief Enumeration of motoman-specific message types.
 *        See simple_message.h for a listing of "standard" message types
 */
namespace MotomanMsgTypes
{
enum MotomanMsgType
{
  MOTOMAN_MSG_BEGIN = 2000,
  MOTOMAN_MOTION_CTRL = 2001,
  MOTOMAN_MOTION_REPLY = 2002,
  ROS_MSG_MOTO_JOINT_TRAJ_PT_FULL_EX = 2016, // This is similar to the "Dynamic Joint Point" in REP I0001
  ROS_MSG_MOTO_JOINT_FEEDBACK_EX = 2017, //Similar to Dynamic Joint State on the REP I0001
};
}
typedef MotomanMsgTypes::MotomanMsgType MotomanMsgType;
}  // namespace simple_message
}  // namespace industrial

#endif  // MOTOMAN_DRIVER_SIMPLE_MESSAGE_MOTOMAN_SIMPLE_MESSAGE_H
