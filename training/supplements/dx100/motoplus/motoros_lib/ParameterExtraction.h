/* ParameterExtraction.h - Parameter Extraction definitions header file */

/*
* Software License Agreement (BSD License) 
*
* Copyright (c) 2011, Yaskawa America, Inc.
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

#ifndef _INC_GETMOTOMANPARAMETERS_H
#define _INC_GETMOTOMANPARAMETERS_H

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_PARAM_BUFF 4000 //4k
#define TAG_BUFF_LEN 10

typedef struct
{
	float PtoR[MAX_PULSE_AXES]; //Array to store PULSE TO RADIAN conversion factors for each axes
}GB_PULSE_TO_RAD;

typedef struct
{
	BOOL  bValid;			//TRUE if ulSourceAxis != 0
	INT32 ulSourceAxis;		
	INT32 ulCorrectionAxis;	 
	float fCorrectionRatio;	
} FB_AXIS_CORRECTION;

typedef struct
{
	FB_AXIS_CORRECTION  correction[MAX_PULSE_AXES];
} FB_PULSE_CORRECTION_DATA;

typedef struct
{
	UINT32	qtyOfOutFiles;				
	UINT32	qtyOfHighPriorityTasks;		
	UINT32	qtyOfNormalPriorityTasks;	
} TASK_QTY_INFO;
	
typedef struct
{
	UINT16 periodInMilliseconds;
} GP_INTERPOLATION_PERIOD;

typedef struct
{
	UINT32	maxIncrement[MAX_PULSE_AXES];
} MAX_INCREMENT_INFO;

typedef struct
{
	int ctrlGrp;				//Robot control group
	int IpCycleInMilliseconds;	//Interpolation Cycle in milliseconds
	MAX_INCREMENT_INFO info;	//Maximum increment per interpolation cycle
} MAX_INC_PIPC;
	
/******************************************************************************/
/* << 4 >>                                                              	  */
/* Function name : STATUS GP_getPulseToRad()								  */
/* Functionality : Gets the Pulse to radians conversion factors				  */
/* Parameter	 : int ctrlGrp - Robot control to fetch data	[IN]		  */
/*				   GB_PULSE_TO_RAD *PulseToRad -array of conversion data [OUT]*/
/* Return value	 : Success = OK 											  */
/*				 : Failure = NG												  */	
/******************************************************************************/
extern STATUS 	GP_getPulseToRad(int ctrlGrp, GB_PULSE_TO_RAD *PulseToRad);

/******************************************************************************/
/* << 6 >>                                                              	  */
/* Function name : int  GP_getNumberOfAxes()								  */
/* Functionality : Retrieves the Number of Axes								  */
/* Parameter	 : NONE														  */
/* Return value	 : Success = Number of Axes									  */
/*				 : Failure = -1												  */	
/******************************************************************************/
extern int  	GP_getNumberOfAxes(int ctrlGrp);
	
/******************************************************************************/
/* << 11 >>                                                             	  */
/* Function name : STATUS GetFBPulseCorrection()							  */
/* Functionality : Get all the pulse correction data for required axes		  */
/* Parameter	 : int ctrlGrp - Robot control to fetch data [IN]			  */
/*				   FB_PULSE_CORRECTION_DATA * correctionData[OUT]			  */
/* Return value	 : Success = OK 											  */
/*				 : Failure = NG												  */
/******************************************************************************/
extern STATUS GP_getFBPulseCorrection(int ctrlGrp, FB_PULSE_CORRECTION_DATA *correctionData);
	
/******************************************************************************/
/* << 12 >>                                                             	  */
/* Function name : STATUS GP_getQtyOfAllowedTasks()							  */
/* Functionality : No.of MotoPlus tasks that can be started concurrently  	  */
/* Parameter	 : TASK_QTY_INFO *taskInfo [OUT]				  			  */
/* Return value	 : Success = OK 											  */
/*				 : Failure = NG												  */
/******************************************************************************/
extern STATUS GP_getQtyOfAllowedTasks(TASK_QTY_INFO *taskInfo);

/******************************************************************************/
/* << 13 >>                                                             	  */
/* Function name : STATUS GP_getInterpolationPeriod()						  */
/* Functionality : No.of millisecs b/w each tick of the interpolation-clock	  */
/* Parameter	 : UINT16 *periodInMilliseconds [OUT]						  */
/* Return value	 : Success = OK 											  */
/*				 : Failure = NG												  */	
/******************************************************************************/
extern STATUS GP_getInterpolationPeriod(UINT16* periodInMilliseconds);

/******************************************************************************/
/* << 14 >>                                                             	  */
/* Function name : STATUS GP_getMaxIncPerIpCycle()							  */
/* Functionality : Max increment the arm is capable of(limited by governor)	  */
/* Parameter	 : int ctrlGrp - Robot control to fetch data [IN]			  */
/*				   int interpolationPeriodInMilliseconds - obtained from GP_getInterpolationPeriod [IN] */
/*				   MAX_INCREMENT_INFO *mip [OUT]	   		 				  */
/* Return value	 : Success = OK 											  */
/*				 : Failure = NG												  */	
/******************************************************************************/
extern STATUS GP_getMaxIncPerIpCycle(int ctrlGrp, int interpolationPeriodInMilliseconds, MAX_INCREMENT_INFO *mip);

/******************************************************************************/
/* << 15 >>                                                             	  */
/* Function name : GP_getGovForIncMotion()									  */
/* Functionality : Percentage Limit of the max-increment					  */
/* Parameter	 : int ctrlGrp 				[IN]			  				  */
/* Return value	 : Success = percentage limit Of MaxSpeed					  */
/*				 : Failure = -1												  */	
/******************************************************************************/
extern float GP_getGovForIncMotion(int ctrlGrp);

#ifdef __cplusplus
}
#endif

#endif
