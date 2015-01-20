// SimpleMessage.c
//
// History:
// 06/12/2013: Fix reply to ROS_MSG_JOINT_TRAJ_PT_FULL message
// 06/12/2013: Release v.1.0.1
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

#include "MotoPlus.h"
#include "ParameterExtraction.h"
#include "CtrlGroup.h"
#include "SimpleMessage.h"
#include "Controller.h"

//-----------------------
// Function Declarations
//-----------------------

//-----------------------
// Function implementation
//-----------------------

// Creates a simple message of type: ROS_MSG_JOINT_FEEDBACK = 15
// Simple message containing a the current joint position
// of the specified control group
int Ros_SimpleMsg_JointFeedback(CtrlGroup* ctrlGroup, SimpleMsg* sendMsg)
{
	int bRet;
	long pulsePos[MAX_PULSE_AXES];
	
	//initialize memory
	memset(sendMsg, 0x00, sizeof(SimpleMsg));
	
	// set prefix: length of message excluding the prefix
	sendMsg->prefix.length = sizeof(SmHeader) + sizeof(SmBodyJointFeedback);
	
	// set header information
	sendMsg->header.msgType = ROS_MSG_JOINT_FEEDBACK;
	sendMsg->header.commType = ROS_COMM_TOPIC;
	sendMsg->header.replyType = ROS_REPLY_INVALID;
	
	// set body
	sendMsg->body.jointFeedback.groupNo = ctrlGroup->groupNo;
	sendMsg->body.jointFeedback.validFields = 2;
	
	bRet = Ros_CtrlGroup_GetFBPulsePos(ctrlGroup, pulsePos);
	if(bRet!=TRUE)
		return 0;
				
	Ros_CtrlGroup_ConvertToRosPos(ctrlGroup, pulsePos, sendMsg->body.jointFeedback.pos);

	// For testing
	//bRet = Ros_CtrlGroup_GetPulsePosCmd(ctrlGroup, pulsePos);
	//if(bRet!=TRUE)
	//	return 0;
	//	
	//Ros_CtrlGroup_ConvertToRosPos(ctrlGroup, pulsePos, sendMsg->body.jointFeedback.vel);
	// End testing
	
	return(sendMsg->prefix.length + sizeof(SmPrefix));
}

// Initialize header for a simple message of type: ROS_MSG_MOTO_JOINT_FEEDBACK_EX = 17
void Ros_SimpleMsg_JointFeedbackEx_Init(int numberOfGroups, SimpleMsg* sendMsg)
{	
	//initialize memory
	memset(sendMsg, 0x00, sizeof(SimpleMsg));
	
	// set prefix: length of message excluding the prefix
	sendMsg->prefix.length = sizeof(SmHeader) + sizeof(SmBodyJointFeedbackEx);
	
	// set header information
	sendMsg->header.msgType = ROS_MSG_MOTO_JOINT_FEEDBACK_EX;
	sendMsg->header.commType = ROS_COMM_TOPIC;
	sendMsg->header.replyType = ROS_REPLY_INVALID;
	
	// set body
	sendMsg->body.jointFeedbackEx.numberOfValidGroups = numberOfGroups;
}

// Copy data from a standard feedback message to the extended feedback message.  This
// function should be called multiple times to build a message for all control groups.
int Ros_SimpleMsg_JointFeedbackEx_Build(int groupIndex, SimpleMsg* src_msgFeedback, SimpleMsg* dst_msgExtendedFeedback)
{
	memcpy(&dst_msgExtendedFeedback->body.jointFeedbackEx.jointTrajPtData[groupIndex],
		   &src_msgFeedback->body.jointFeedback,
		   sizeof(SmBodyJointFeedback));
	
	return(dst_msgExtendedFeedback->prefix.length + sizeof(SmPrefix));
}


// Creates a simple message of type MOTO_MOTION_REPLY to reply to a received message 
// result code and subcode indication result of the processing of the received message
// 06/12/2013: Modified to fix reply to ROS_MSG_JOINT_TRAJ_PT_FULL message
int Ros_SimpleMsg_MotionReply(SimpleMsg* receiveMsg, int result, int subcode, SimpleMsg* replyMsg, int ctrlGrp)
{
	//initialize memory
	memset(replyMsg, 0x00, sizeof(SimpleMsg));
	
	// set prefix: length of message excluding the prefix
	replyMsg->prefix.length = sizeof(SmHeader) + sizeof(SmBodyMotoMotionReply);

	// set header information of the reply
	replyMsg->header.msgType = ROS_MSG_MOTO_MOTION_REPLY;
	replyMsg->header.commType = ROS_COMM_SERVICE_REPLY;
	replyMsg->header.replyType = ROS_REPLY_SUCCESS;
	
	// set reply body
	if(receiveMsg->header.msgType == ROS_MSG_MOTO_MOTION_CTRL)
	{
		replyMsg->body.motionReply.groupNo = ctrlGrp;
		replyMsg->body.motionReply.sequence = receiveMsg->body.motionCtrl.sequence;
		replyMsg->body.motionReply.command = receiveMsg->body.motionCtrl.command;
	}
	else if(receiveMsg->header.msgType == ROS_MSG_JOINT_TRAJ_PT_FULL)
	{
		replyMsg->body.motionReply.groupNo = ctrlGrp;
		replyMsg->body.motionReply.sequence = receiveMsg->body.jointTrajData.sequence;
		replyMsg->body.motionReply.command = ROS_MSG_JOINT_TRAJ_PT_FULL;
	}
	else if (receiveMsg->header.msgType == ROS_MSG_MOTO_JOINT_TRAJ_PT_FULL_EX)
	{
		replyMsg->body.motionReply.groupNo = ctrlGrp;
		replyMsg->body.motionReply.sequence = receiveMsg->body.jointTrajDataEx.sequence;
		replyMsg->body.motionReply.command = ROS_MSG_MOTO_JOINT_TRAJ_PT_FULL_EX;
	}
	else
	{
		replyMsg->body.motionReply.groupNo = -1;
		replyMsg->body.motionReply.sequence = -1;
		replyMsg->body.motionReply.command = receiveMsg->header.msgType;
	}

	replyMsg->body.motionReply.result = result;
	replyMsg->body.motionReply.subcode = subcode;
	
	return(replyMsg->prefix.length + sizeof(SmPrefix));
}

#ifdef DEBUG
// function to dump data structure for debugging
void Ros_SimpleMsg_DumpTrajPtFull(SmBodyJointTrajPtFull* data)
{
	printf("Dumping SmBodyJointTrajPtFull:\r\n");
	printf("  groupNo=%d\r\n", data->groupNo);
	printf("  sequence=%d\r\n", data->sequence);
	printf("  validFields=%d\r\n", data->validFields);
	printf("  time=%%.5f\r\n", data->time);
	printf("  pos: %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f\r\n", 
		data->pos[0], data->pos[1], data->pos[2], data->pos[3], 
		data->pos[4], data->pos[5], data->pos[6]);
	printf("  vel: %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f\r\n", 
		data->vel[0], data->vel[1], data->vel[2], data->vel[3], 
		data->vel[4], data->vel[5], data->vel[6]);
	printf("  acc: %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f\r\n", 
		data->acc[0], data->acc[1], data->acc[2], data->acc[3], 
		data->acc[4], data->acc[5], data->acc[6]);
}
#endif
