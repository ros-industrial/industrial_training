/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
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


#ifndef JOINT_MOTION_HANDLER_H
#define JOINT_MOTION_HANDLER_H

#include "motoPlus.h"
#include "message_handler.h"
#include "joint_message.h"
#include "p_var_q.h"

namespace motoman
{
namespace joint_motion_handler
{

/**
 * \brief Message handler that handles joint motion requests to the controller
 */
//* MessageHandler

/**
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 *
*/
 
/*
Motoplus Functional Specs:

Upon receiving...
1st point - enable motion, start job, add point (increment buffer index), 
Nth point - add point (increment buffer index)
end of trajectory - wait until buffer size = 0, disable motion, stop job, reset buffer indicies
motion stop - disable motion, stop job

INFORM Functional Specs:

start job - reset buffer index
non-empty buffer - execute motion, increment motion pointer
empty buffer - do not execute any motion (wait for points)
end of position variables (jump to beginning position)
 
TODO: WHAT TO DO IF THE END OF TRAJECTORY FLAG HAS BEEN RECEIVED AND
THEN A MOTION CANCEL HAS BEEN RECEIVED.  MIGHT WANT TO CONSIDER USING
AN EXTRA VARIABLE ON THE CONTROLLER FOR THE INFORM JOB TO MONITOR
*/


class JointMotionHandler : public industrial::message_handler::MessageHandler
{

public:

JointMotionHandler(void);
~JointMotionHandler(void);

  /**
* \brief Class initializer
*
* \param connection simple message connection that will be used to send replies.
*
* \return true on success, false otherwise (an invalid message type)
*/
bool init(industrial::smpl_msg_connection::SmplMsgConnection* connection);

  /**
* \brief Class initializer (Direct call to base class with the same name)
* I couldn't get the "using" form to work/
*
* \param connection simple message connection that will be used to send replies.
*
* \return true on success, false otherwise (an invalid message type)
*/
bool init(int msg_type, industrial::smpl_msg_connection::SmplMsgConnection* connection)
{ return MessageHandler::init(msg_type, connection);};


private:


// Servo power variables
MP_SERVO_POWER_SEND_DATA servo_power_data;
MP_STD_RSP_DATA servo_power_error;

// Job variables
MP_START_JOB_SEND_DATA job_start_data;
MP_CUR_JOB_SEND_DATA job_reset_data;
MP_STD_RSP_DATA job_reset_error;
MP_STD_RSP_DATA job_error;


// Hold variables
MP_HOLD_SEND_DATA hold_data;
MP_STD_RSP_DATA hold_error;
	
motoman::p_var_q::PVarQ pVarQ; 
 
 /**
  * \brief Poll delay (in ticks) when querying the motoplus api.
  */
 static const int MP_POLL_TICK_DELAY = 10;
 
 /**
  * \brief Poll delay (in ticks) when querying the motion buffer
  */
 static const int BUFFER_POLL_TICK_DELAY = 1000;
 
 //TODO: motion and job flags are just internal state variables, we may
 //want to make them query the appropriate motoplus API calls instead.
 //This should just require rewriting the access functions below.
 /**
  * \brief True if motion enabled
  */
 bool motionEnabled;
 
 /**
  * \brief True if job started
  */
 bool jobStarted;
 
   /**
* \brief return true if motion is enabled (Based on internal class state
* not MotoPlus call)
*
* \return true if motion enabled
*/ 
bool isMotionEnabled(void) {return motionEnabled;};

   /**
* \brief return true if jos has been started (Based on internal class state
* not MotoPlus call)
*
* \return true if job has been stared
*/ 
bool isJobStarted(void) {return jobStarted;};
 
 /**
  * \brief Callback executed upon receiving a joint message
  *
  * \param in incoming message
  *
  * \return true on success, false otherwise
  */
 bool internalCB(industrial::simple_message::SimpleMessage & in);
 
 
 /**
  * \brief Handles motion interface based on type of joint message passed
  *
  * \param in incoming joint message
  *
  */
 void motionInterface(industrial::joint_message::JointMessage & jMsg);
 
 
  /**
* \brief Enables motion on the robot controller.  Turns on servo power, turns
* off hold status
*
*/ 
void enableMotion(void);

  /**
* \brief Disables motion on the robot controller.  Turns off servo power, turns
* on hold status
*
*/ 
void disableMotion(void);

 /**
* \brief Starts motion job on the controller.  Enables motion (Job cannot be started
* if motion is not enabled).
*
*/ 
void startMotionJob(void);
	
 /**
* \brief Stops motion job on the controller.  Disables motion
*
*/ 
void stopMotionJob(void);
	
};

}//joint_motion_handler
}//industrial


#endif /* JOINT_MOTION_HANDLER_H */