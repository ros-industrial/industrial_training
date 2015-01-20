// StateServer.c
//
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

#include "MotoPlus.h"
#include "ParameterExtraction.h"
#include "CtrlGroup.h"
#include "SimpleMessage.h"
#include "Controller.h"
#include "StateServer.h"

//-----------------------
// Function Declarations
//-----------------------
void Ros_StateServer_StartNewConnection(Controller* controller, int sd);
void Ros_StateServer_SendState(Controller* controller);
BOOL Ros_StateServer_SendMsgToAllClient(Controller* controller, SimpleMsg* sendMsg, int msgSize);

//-----------------------
// Function implementation
//-----------------------

//-----------------------------------------------------------------------
// Start the tasks for a new state server connection:
// - WaitForSimpleMsg: Task that waits to receive new SimpleMessage
// - AddToIncQueueProcess: Task that take data from a message and generate Incmove  
//-----------------------------------------------------------------------
void Ros_StateServer_StartNewConnection(Controller* controller, int sd)
{
	int connectionIndex;

	printf("Starting new connection to the State Server\r\n");
	
    //look for next available connection slot
    for (connectionIndex = 0; connectionIndex < MAX_STATE_CONNECTIONS; connectionIndex++)
    {
      	if (controller->sdStateConnections[connectionIndex] == INVALID_SOCKET)
      	{
      		//Start the new connection in a different task.
       		//Each task's memory will be unique IFF the data is on the stack.
       		//Any global or heap stuff will not be unique.
	    	controller->sdStateConnections[connectionIndex] = sd;
		    	
	    	// If not started
			if(controller->tidStateSendState == INVALID_TASK)
			{
	   			//start task that will send the controller state
				controller->tidStateSendState = mpCreateTask(MP_PRI_TIME_NORMAL, MP_STACK_SIZE, 
											(FUNCPTR)Ros_StateServer_SendState,
			 								(int)controller, 0, 0, 0, 0, 0, 0, 0, 0, 0);
	
				//set feedback signal
				if(controller->tidStateSendState != INVALID_TASK)
					Ros_Controller_SetIOState(IO_FEEDBACK_STATESERVERCONNECTED, TRUE);
			}
	        
	        break;
	    }
    }
        
    if (connectionIndex == MAX_STATE_CONNECTIONS)
    {
       	printf("Too many State server connections... not accepting last attempt.\r\n");
       	mpClose(sd);
    }
}


//-----------------------------------------------------------------------
// Send state (robot position) as long as there is an active connection
//-----------------------------------------------------------------------
void Ros_StateServer_SendState(Controller* controller)
{
	BOOL bHasConnections;
	int groupNo;
	SimpleMsg sendMsg;
	SimpleMsg sendMsgFEx;
	int msgSize, fexMsgSize;
	BOOL bOkToSendExFeedback;

	bHasConnections = TRUE;
	
	printf("Starting State Server Send State task\r\n");
	printf("Controller number of group = %d\r\n", controller->numGroup);
	
	while(bHasConnections)
	{
		Ros_SimpleMsg_JointFeedbackEx_Init(controller->numGroup, &sendMsgFEx);
		bOkToSendExFeedback = TRUE;

		// Send feedback position for each control group
		for(groupNo=0; groupNo < controller->numGroup; groupNo++)
		{
			msgSize = Ros_SimpleMsg_JointFeedback(controller->ctrlGroups[groupNo], &sendMsg);
			fexMsgSize = Ros_SimpleMsg_JointFeedbackEx_Build(groupNo, &sendMsg, &sendMsgFEx);
			if(msgSize > 0)
			{
				if (bHasConnections)
					bHasConnections = Ros_StateServer_SendMsgToAllClient(controller, &sendMsg, msgSize);
			}
			else
			{
				printf("Ros_SimpleMsg_JointFeedback returned a message size of 0\r\n");
				bOkToSendExFeedback = FALSE;
			}
		}

		if (controller->numGroup < 2) //only send the ROS_MSG_MOTO_JOINT_FEEDBACK_EX message if we have multiple control groups
			bOkToSendExFeedback = FALSE;

		if (bHasConnections && bOkToSendExFeedback) //send extended-feedback message
			bHasConnections = Ros_StateServer_SendMsgToAllClient(controller, &sendMsgFEx, fexMsgSize);

		// Send controller/robot status
		if(bHasConnections)
		{
			msgSize = Ros_Controller_StatusToMsg(controller, &sendMsg);
			if(msgSize > 0)
			{
				bHasConnections = Ros_StateServer_SendMsgToAllClient(controller, &sendMsg, msgSize);
			}
		}
		mpTaskDelay(STATE_UPDATE_MIN_PERIOD);
	}
	
	// Terminate this task
	controller->tidStateSendState = INVALID_TASK;
	Ros_Controller_SetIOState(IO_FEEDBACK_STATESERVERCONNECTED, FALSE);
	printf("State Server Send State task was terminated\r\n");
	mpDeleteSelf;
}


//-----------------------------------------------------------------------
// Send state message to all active connections
// return TRUE if message was send to at least one client
//-----------------------------------------------------------------------
BOOL Ros_StateServer_SendMsgToAllClient(Controller* controller, SimpleMsg* sendMsg, int msgSize)
{
	int index;
	int ret;
	BOOL bHasConnections = FALSE;
	
	// Check all active connections
	for(index = 0; index < MAX_STATE_CONNECTIONS; index++)
	{
		if(controller->sdStateConnections[index] != INVALID_SOCKET)
		{
			ret = mpSend(controller->sdStateConnections[index], (char*)(sendMsg), msgSize, 0);
			if(ret <= 0)
			{
				printf("StateServer Send failure.  Closing state server connection.\r\n");
				mpClose(controller->sdStateConnections[index]);
				controller->sdStateConnections[index] = INVALID_SOCKET;
			}
			else
			{
				bHasConnections = TRUE;
			}
		}
	}
	return bHasConnections;
}
