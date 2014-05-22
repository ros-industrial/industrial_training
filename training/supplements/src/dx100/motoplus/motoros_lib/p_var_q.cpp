/*
* Software License Agreement (BSD License) 
*
* Copyright (c) 2011, Yaskawa America, Inc.
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
*       * Neither the name of the Yaskawa America, Inc., nor the names 
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

#include "p_var_q.h"
#include "controller.h"
#include "joint_data.h"
#include "joint_motion_handler.h"
#include "ros_conversion.h"
#include "log_wrapper.h"

using motoman::controller::Controller;
using motoman::joint_motion_handler;
using motoman::ros_conversion;
using industrial::joint_data;


namespace motoman
{
namespace p_var_q
{


PVarQ::PVarQ()
{	
  // TODO: Should check constants such as Q_SIZE, MOTION_POINTER & BUFFER_POINTER for
  // consitency
  
  // Checking class constants for consistancy
  STATIC_ASSERT(QSIZE_ > PT_LOOK_AHEAD_, QSIZE_NOT_GREATER_THAN_LOOK_AHEAD);
  STATIC_ASSERT(MOTION_POINTER_ > QSIZE_, MOTION_PONTER_NOT_GREATER_THAN_QSIZE);
  STATIC_ASSERT(BUFFER_POINTER_ > QSIZE_, BUFFER_PONTER_NOT_GREATER_THAN_QSIZE);
  STATIC_ASSERT(MIN_BUF_START_POINTER_ > QSIZE_, MIN_BUFF_START_PONTER_NOT_GREATER_THAN_QSIZE);
  
}

PVarQ::~PVarQ(void)
{
}


void PVarQ::init(industrial::joint_data::JointData & point, double velocity_percent)
{
  // Reseting the position buffer.  This should make it eaiser to catch any bugs on the
  // motoman side.
  industrial::joint_data::JointData empty;
  for(int i = 0; i < this->posVarQueueSize(); i++)
  {
    this->setPosition(i, empty, 0);
  }
  
  // Seed the intial point - this is required because upon startup, the indexes are zero and
  // therefore the intial point never gets set.
  setPosition(0, point, this->TEMP_getVelocityPercent());
  
  // Set the minium buffer size
  Controller::setInteger(MIN_BUF_START_POINTER_, PT_LOOK_AHEAD_);
}

void PVarQ::addPoint(industrial::joint_data::JointData & joints, double velocity_percent)
{

  // Wait until buffer is not full
  
  LOG_DEBUG("Checking for buffer full");
  while(this->bufferFull()) {
      mpTaskDelay(this->BUFFER_POLL_TICK_DELAY_);
  };
  LOG_DEBUG("Buffer ready to accept new points");
  
  setNextPosition(joints, this->TEMP_getVelocityPercent());
  incBufferIndex();
	
}

int PVarQ::bufferSize()
{
  int motionIdx = this->getMotionIndex();
  int bufferIdx = this->getBufferIndex();
  int rtn = 0;
  
  if (motionIdx > bufferIdx)
  {
    LOG_ERROR("Motion index: %d is greater than Buffer index: %d, returning empty buffer size", motionIdx, bufferIdx);
    rtn = 0;
  }
  else
  {
    rtn = bufferIdx - motionIdx;
  }
  
  return rtn;  
}
    

  
int PVarQ::getMotionPosIndex()
{
  return (this->getMotionIndex() % this->posVarQueueSize());
}
  
int PVarQ::getBufferPosIndex()
{
  return (this->getBufferIndex() % this->posVarQueueSize());
}

int PVarQ::getNextBufferPosIndex()
{
  return ((this->getBufferIndex() + 1) % this->posVarQueueSize());
}
    
bool PVarQ::bufferFull()
{
  bool rtn = false;
  int bufferSize = this->bufferSize();
  int maxBufferSize = this->maxBufferSize();
  if (bufferSize >= maxBufferSize)
  {
    rtn = true;
  }
  return rtn;
}

bool PVarQ::bufferEmpty()
{
  bool rtn = false;
  if (this->bufferSize() <= 0)
  {
    LOG_DEBUG("Buffer is empty");
    rtn = true;
  }
  return rtn;
}

void PVarQ::incBufferIndex()
{
  int bufferIdx = this->getBufferIndex();
  
  LOG_DEBUG("Incrementing buffer index from %d to %d", bufferIdx, bufferIdx + 1); 
  Controller::setInteger(BUFFER_POINTER_, bufferIdx + 1);
}


void PVarQ::setNextPosition(industrial::joint_data::JointData & point, double velocity_percent)
{
  setPosition(this->getNextBufferPosIndex(), point, velocity_percent); 
}


void PVarQ::setPosition(int index, industrial::joint_data::JointData & point, 
    double velocity_percent)
{
  const double VELOCITY_CONVERSION = 100.0;
  int convertedVelocity = 0;
  
  LOG_DEBUG("Setting joint position, index: %d", index);

  while (!Controller::setJointPositionVar(index, point)) {
    LOG_ERROR("Failed set position variable, index: %d, retrying...", index);
    mpTaskDelay(this->VAR_POLL_TICK_DELAY_);
  };
  
  convertedVelocity = (int)(velocity_percent * VELOCITY_CONVERSION);
  LOG_DEBUG("Converting percent velocity: %g to motoman integer value: %d", 
    velocity_percent, convertedVelocity);  
  LOG_DEBUG("Setting velocity, index: %d, value: %d", index, convertedVelocity);
  Controller::setInteger(index, convertedVelocity);
}



}//namespace p_var_q
}//namespace motoman