/*
* Software License Agreement (BSD License) 
*
* Copyright (c) 2011, Southwest Research Institute
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 	* Redistributions of source code must retain the above copyright
* 	notice, this list of conditions and the following disclaimer.
* 	* Redistributions in binary form must reproduce the above copyright
* 	notice, this list of conditions and the following disclaimer in the
* 	documentation and/or other materials provided with the distribution.
* 	* Neither the name of the Southwest Research Institute, nor the names 
*	of its contributors may be used to endorse or promote products derived
*	from this software without specific prior written permission.
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


#include "joint_motion_handler.h"
#include "joint_message.h"
#include "log_wrapper.h"


using namespace industrial::joint_message;
using namespace industrial::joint_data;
using namespace industrial::simple_message;

namespace motoman
{
namespace joint_motion_handler
{

#define JOBNAME "PVARQ20VV"

JointMotionHandler::JointMotionHandler()
{	
  // Set up job variables
  // start task
  job_start_data.sTaskNo = 0;
  strcpy(job_start_data.cJobName, JOBNAME);
 
  // reset task
  strcpy(job_reset_data.cJobName, JOBNAME);
  
  this->jobStarted = false;
  this->motionEnabled = false;
}

JointMotionHandler::~JointMotionHandler()
{
  this->disableMotion();
  this->stopMotionJob();
}

bool JointMotionHandler::init(industrial::smpl_msg_connection::SmplMsgConnection* connection)
{
  return this->init(StandardMsgTypes::JOINT_POSITION, connection);
}

bool JointMotionHandler::internalCB(industrial::simple_message::SimpleMessage & in)
{
  bool rtn = false;
  JointMessage jMsg;
  SimpleMessage reply;

    if (jMsg.init(in))
    {
        this->motionInterface(jMsg);
    }
    else
    {
    LOG_ERROR("Failed to initialize joint message");
    rtn = false;
    }

    // Send response if requested
    if (CommTypes::SERVICE_REQUEST == in.getCommType())
        if (jMsg.toReply(reply, ReplyTypes::SUCCESS))
        {
            if(this->getConnection()->sendMsg(reply))
            {
                LOG_INFO("Joint ack sent");
                rtn = true;
            }
            else
            {
                LOG_ERROR("Failed to send joint ack");
                rtn = false;
            }
        }
        else
        {
            LOG_ERROR("Failed to generate joint ack message");
            rtn = false;
        }
    
  return rtn;
}


 void JointMotionHandler::motionInterface(JointMessage & jMsg)
 {
//
// Upon receiving...
// 1st point - initialize buffer, enable motion, start job, add point (increment buffer index), 
// Nth point - add point (increment buffer index)
// end of trajectory - wait until buffer size = 0, disable motion, stop job, reset buffer indicies
//motion stop - disable motion, stop job

  JointData joints;
  
   switch (jMsg.getSequence())
    {
      case SpecialSeqValues::END_TRAJECTORY:
         LOG_INFO("Received end trajectory command");
         while(!pVarQ.bufferEmpty())
         {
           LOG_DEBUG("Waiting for motion buffer to empty");
           mpTaskDelay(this->BUFFER_POLL_TICK_DELAY);
         };
         this->stopMotionJob();
           
         break;
         
      case SpecialSeqValues::STOP_TRAJECTORY:
         LOG_WARN("Received stop command, stopping motion immediately");
         this->stopMotionJob();
         break;
         
      default:
        
        joints.copyFrom(jMsg.getJoints());
        if (!(this->isJobStarted()))
        {
          //TODO: The velocity should be set from the message in the future.
          pVarQ.init(joints, 0.0);
          this->startMotionJob();
        }
        else
        {
          pVarQ.addPoint(joints, 0.0);
        }
    }
 }
 
 
void JointMotionHandler::enableMotion(void)
{
  
  LOG_INFO("Enabling motion");
  this->motionEnabled = false;

  servo_power_data.sServoPower = ON;
  while(mpSetServoPower(&servo_power_data, &servo_power_error) == ERROR)
  {
    LOG_ERROR("Failed to turn on servo power, error: %d, retrying...", servo_power_error.err_no);
    mpTaskDelay(this->MP_POLL_TICK_DELAY);
  };
  
  hold_data.sHold = OFF;
  while(mpHold(&hold_data, &hold_error) == ERROR)
  {
    LOG_ERROR("Failed to turn off hold, error: %d, retrying...", hold_error.err_no);
    mpTaskDelay(this->MP_POLL_TICK_DELAY);
  };
  
  this->motionEnabled = true;
}


void JointMotionHandler::disableMotion(void)
{
  LOG_INFO("Disabling motion");
  servo_power_data.sServoPower = OFF;
  while(mpSetServoPower(&servo_power_data, &servo_power_error) == ERROR)
  {
    LOG_ERROR("Failed to turn off servo power, error: %d, retrying...", servo_power_error.err_no);
    mpTaskDelay(this->MP_POLL_TICK_DELAY);
  };
  
  this->motionEnabled = false;
}

void JointMotionHandler::startMotionJob(void)
{

  this->jobStarted = false;
  
  this->enableMotion();
  
  LOG_INFO("Starting motion job");
  while(mpStartJob(&job_start_data, &job_error) == ERROR)
  {
    LOG_ERROR("Failed to start job, error: %d, retrying...", job_error.err_no);
    mpTaskDelay(this->MP_POLL_TICK_DELAY);
  };
  
  LOG_DEBUG("Waiting for indexes to reset");
  // The INFORM job should reset the index counters 
  while( (pVarQ.getMotionPosIndex() != 0) && ( pVarQ.getBufferPosIndex() != 0))
  {
    LOG_ERROR("Waiting for indexes to reset, retying...");
    mpTaskDelay(this->MP_POLL_TICK_DELAY);
  };
  
  LOG_DEBUG("Reset indexes, motion: %d, buffer: %d", pVarQ.getMotionPosIndex(),
    pVarQ.getBufferPosIndex());  
  
  this->jobStarted = true;
}
void JointMotionHandler::stopMotionJob(void)
{  
  LOG_INFO("Stopping motion job");
  this->disableMotion();
  
  // Reseting the job.
  job_reset_data.usJobLine = 1;  //Job lines start at 1
  while(mpSetCurJob(&job_reset_data, &job_reset_error) == ERROR)
  {
    LOG_ERROR("Failed to reset job, error: %d, retrying...", job_reset_error.err_no);
    mpTaskDelay(this->MP_POLL_TICK_DELAY);
  };
  
  this->jobStarted = false;
}	

}//namespace joint_motion_handler
}//namespace industrial


