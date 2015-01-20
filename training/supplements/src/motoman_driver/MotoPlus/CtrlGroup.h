// CtrlGroup.h
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

#ifndef CTRLGROUP_H
#define CTRLGROUP_H

#define Q_SIZE 200
#define Q_LOCK_TIMEOUT 1000
#define	Q_OFFSET_IDX( a, b, c )	(((a)+(b)) >= (c) ) ? ((a)+(b)-(c)) \
				: ( (((a)+(b)) < 0 ) ? ((a)+(b)+(c)) : ((a)+(b)) )
				
typedef struct
{
	LONG time;
	UCHAR frame;
	UCHAR user;
	UCHAR tool;
	LONG inc[MP_GRP_AXES_NUM];
} Incremental_data;

typedef struct
{
	SEM_ID q_lock;
	LONG cnt;
	LONG idx;
	Incremental_data data[Q_SIZE];
} Incremental_q;



// jointMotionData values are in radian and joint order in sequential order 
typedef struct
{
	int flag;
	int time;						// time in millisecond
	float pos[MP_GRP_AXES_NUM];		// position in radians
	float vel[MP_GRP_AXES_NUM];		// velocity in radians/s
	float acc[MP_GRP_AXES_NUM];		// acceleration in radians/s^2
} JointMotionData;


//---------------------------------------------------------------
// CtrlGroup:
// Structure containing all the data related to a control group 
//---------------------------------------------------------------
typedef struct 
{
	int groupNo;								// sequence group number
	int numAxes;								// number of axis in the control group
	MP_GRP_ID_TYPE groupId;						// control group ID
	GB_PULSE_TO_RAD pulseToRad;					// conversion ratio between pulse and radian
	FB_PULSE_CORRECTION_DATA correctionData;  	// compensation for axes coupling
	MAX_INCREMENT_INFO maxInc;					// maximum increment per interpolation cycle
	float maxSpeedRad[MP_GRP_AXES_NUM];			// maximum joint speed in radian/sec
	
	Incremental_q inc_q;						// incremental queue
	long q_time;								// time to which the queue as been processed
	
	JointMotionData jointMotionData;			// joint motion command data in radian
	JointMotionData jointMotionDataToProcess;	// joint motion command data in radian to process
	BOOL hasDataToProcess;						// indicates that there is data to process
	int tidAddToIncQueue;						// ThreadId to add incremental values to the queue
	int timeLeftover_ms;						// Time left over after reaching the end of a trajectory to complet the interpolation period
} CtrlGroup;


//---------------------------------
// External Functions Declaration
//---------------------------------

extern CtrlGroup* Ros_CtrlGroup_Create(int groupNo, float interpolPeriod);

extern BOOL Ros_CtrlGroup_GetPulsePosCmd(CtrlGroup* ctrlGroup, long pulsePos[MAX_PULSE_AXES]);

extern BOOL Ros_CtrlGroup_GetFBPulsePos(CtrlGroup* ctrlGroup, long pulsePos[MAX_PULSE_AXES]);

extern void Ros_CtrlGroup_ConvertToRosPos(CtrlGroup* ctrlGroup, long pulsePos[MAX_PULSE_AXES], float radPos[MAX_PULSE_AXES]);

extern void Ros_CtrlGroup_ConvertToMotoPos(CtrlGroup* ctrlGroup, float radPos[MAX_PULSE_AXES], long pulsePos[MAX_PULSE_AXES]);

extern UCHAR Ros_CtrlGroup_GetAxisConfig(CtrlGroup* ctrlGroup);

extern BOOL Ros_CtrlGroup_IsRobot(CtrlGroup* ctrlGroup);

#endif
