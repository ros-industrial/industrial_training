/* mp_main.c - MotoPlus Test Application for Real Time Process */
// History:
// 06/12/2013: Fix reply to ROS_MSG_JOINT_TRAJ_PT_FULL message
// 06/12/2013: Release v.1.0.1
// 07/15/2013: Added DX100 compiler option
//     		   Change "REMOTE" mode I/O signal to #80011
//			   Listen for skill send
// 08/14/2013: Check initialization success and added extra I/O Feedback 
//             Release v.1.1.1
// June 2014:	Release v1.2.0
//				Add support for multiple control groups.
//				Add support for DX200 controller.
/*
* Software License Agreement (BSD License) 
*
* Copyright (c) 2013, Yaskawa America, Inc.
* All rights reserved.
*
* Redistribution and use in binary form, with or without modification,
* is permitted provided that the following conditions are met:
*
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

#include "motoPlus.h"
#include "ParameterExtraction.h"
#include "CtrlGroup.h"
#include "SimpleMessage.h"
#include "Controller.h"
#include "StateServer.h"
#include "MotionServer.h"


#ifdef DEBUG
	#warning Debug messages in MotoPlus *will* affect application performance (disable this in SimpleMessage.h
#endif

//GLOBAL DATA DEFINITIONS

void RosInitTask();
int RosInitTaskID;

void mpUsrRoot(int arg1, int arg2, int arg3, int arg4, int arg5, int arg6, int arg7, int arg8, int arg9, int arg10)
{

//#ifdef DX100
	mpTaskDelay(10000);  // 10 sec. delay to enable DX100 system to complete initialization
//#endif
	
	//Creates and starts a new task in a seperate thread of execution.
	//All arguments will be passed to the new task if the function
	//prototype will accept them.
	RosInitTaskID = mpCreateTask(MP_PRI_TIME_NORMAL, MP_STACK_SIZE, (FUNCPTR)RosInitTask,
						arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10);
									
	//Ends the initialization task.
	mpExitUsrRoot;
}

void RosInitTask()
{
	Controller ros_controller;

	if(!Ros_Controller_Init(&ros_controller))
	{
		//set feedback signal to notify failure
		Ros_Controller_SetIOState(IO_FEEDBACK_FAILURE, TRUE);
		mpDeleteSelf;
		return;
	}

	ros_controller.tidConnectionSrv = mpCreateTask(MP_PRI_TIME_NORMAL, MP_STACK_SIZE, 
						(FUNCPTR)Ros_Controller_ConnectionServer_Start,
						(int)&ros_controller, 0, 0, 0, 0, 0, 0, 0, 0, 0);
		
#ifdef DX100
	// DX100 need to execute a SKILLSEND command prior to the WAIT in order for the 
	// incremental motion function to work.  These tasks monitor for those commands
	// This supports a maximum of two robots which should be assigned to slave id
	// MP_SL_ID1 and MP_SL_ID2 respectively.
	
	// first robot
	ros_controller.RosListenForSkillID[0] = mpCreateTask(MP_PRI_TIME_NORMAL, MP_STACK_SIZE, 
						(FUNCPTR)Ros_Controller_ListenForSkill,
						(int)&ros_controller, MP_SL_ID1, 0, 0, 0, 0, 0, 0, 0, 0);
	// if second robot
	if(ros_controller.numRobot > 1)
	{
		ros_controller.RosListenForSkillID[1] = mpCreateTask(MP_PRI_TIME_NORMAL, MP_STACK_SIZE, 
							(FUNCPTR)Ros_Controller_ListenForSkill,
							(int)&ros_controller, MP_SL_ID2, 0, 0, 0, 0, 0, 0, 0, 0);	
	}
	else
	{
		ros_controller.RosListenForSkillID[1] = INVALID_TASK;
	}
#endif
		
	// start loop to monitor controller state
	FOREVER
	{
		// Check controller status
		if (!Ros_Controller_StatusUpdate(&ros_controller))
			puts("Failed to update controller status.  Check robot parameters.");
	
		mpTaskDelay(CONTROLLER_STATUS_UPDATE_PERIOD);
	}
}



