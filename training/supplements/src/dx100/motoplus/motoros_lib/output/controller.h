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

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "motoPlus.h"
#include "ParameterExtraction.h"  // new motoPlus library
#include "joint_data.h"
#include "robot_status.h"

namespace motoman
{
namespace controller
{

// bitmask used to track which parameters have been initialized
typedef enum
{
  GP_GETNUMBEROFAXES        = 0x01,
  GP_GETPULSETORAD          = 0x02,
  GP_GETFBPULSECORRECTION   = 0x04,
  GP_GETQTYOFALLOWEDTASKS   = 0x08,
  GP_GETINTERPOLATIONPERIOD = 0x10,
  GP_GETMAXINCPERIPCYCLE    = 0x20,
  GP_GETGOVFORINCMOTION     = 0x40,
} param_func_t;

/**
 * \brief Structure for storing control-group parameters
 */
struct ctrl_grp_param_t
{
  int initialized;
  int num_axes;
  float pulses_per_rad[MAX_PULSE_AXES];
  FB_AXIS_CORRECTION pulse_correction[MAX_PULSE_AXES];
  int max_incr_per_cycle[MAX_PULSE_AXES];
  float incr_motion_limit;
  
  ctrl_grp_param_t() : initialized(0) {}  // default constructor to guarantee un-initialized at creation
};

/**
 * \brief Structure for storing system parameters
 */
struct sys_param_t
{
  int initialized;
  TASK_QTY_INFO tasks;
  UINT16 interp_period;
  
  sys_param_t() : initialized(0) {}  // default constructor to guarantee un-initialized at creation
};


/**
 * \brief Spedifies the drive name for DRAM (This should be defined in motoPlus.h)
 */
#define MP_DRAM_DEV_DOS "MPRAM1:"  //This macro is supposed to be defined in motoPlus.h

/**
 * \brief Class encapsulates the motoman controller interface.  It also handles
 * higher level functions such as maintining the current motion state.  This class
 * is meant to be a Singleton, but that is not explicitly enforced.  Only one instance
 * of this object should be instantiated.
 */
//* Controller
/**
 *
 *
 * THIS CLASS IS NOT THREAD-SAFE (the methods that simply wrap motoplus calls can
 * be considered thread safe).  Some internal class state data is maintained, which
 * is not thread safe.
 * TODO: This class will likely be shared between threads, it should be made thread
 * safe.
 *
 */

class Controller
{
public:

/**
  * \brief Default controller
  *
  */
 Controller();
 
 /**
  * \brief Destructor (disables motion and stops current job);
  *
  */
 ~Controller();
 
  /**
  * \brief Reads key values from parameter data-list.
  *
  * \param ctrl_grp Robot Control Group to retrieve. Zero-based (i.e. 0 = Control Group 1)
  *
  * \return true if parameters successfully read
  */
 static bool initParameters(int ctrl_grp=0);
 
 /**
  * \brief Read integer data from the controller integer data table.  Function
  * blocks until data is read
  *
  * \param index index in data table
  *
  * \return integer value
  */
static int getInteger(int index);


/**
  * \brief Write integer data to the controller integer data table.  Function
  * blocks until data is written
  *
  * \param index index in data table
  * \param value value to write
  */
static void setInteger(int index, int value);

/**
  * \brief Write position-variable data to the controller data table.  Function
  * blocks until data is written
  *
  * \param index index in data table
  * \param ros_joints joint positions to write (ROS-order, radians)
  *
  * \return true if variable set successfully
  */
static bool setJointPositionVar(int index, industrial::joint_data::JointData ros_joints);

 /**
  * \brief Utility function for setting a digital output in the
  * Universal output data table (most IO is accessible there).
  *
  * \param bit_offset bit offset in data table (0-2047)
  * \param value in incoming message
  *
  */
 void setDigitalOut(int bit_offset, bool value);
 
 /**
  * \brief Utility function for waiting for a digital input
  * in the Universal input data tabel (most IO is accessible there).
  *
  * \param bit_offset bit offset in data table (0-2047)
  * \param wait_value in incoming message
  *
  */
 void waitDigitalIn(int bit_offset, bool wait_value);
 
 /**
  * \brief Get the actual Joint Position from the robot encoders
  * NOTE: use getCmdJointPos to get the commanded joint positions
  *
  * \param pos array to hold joint positions (in radians)
  *
  * \return true if positions retrieved successfully
  */
 static bool getActJointPos(float* pos);
  
  
  /**
  * \brief Gets the current robot status from the controller.
  *
  * \param status robot status data structure.
  *
  * \return true if robot status successfully retrieved.
  */
 bool getStatus(industrial::robot_status::RobotStatus & status);
 
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
void startMotionJob(char* job_name);
	
 /**
* \brief Stops motion job on the controller.  Disables motion
*
*/ 
void stopMotionJob(char* job_name);

 /**
* \brief Stops motion job on the controller.  Disables motion
*
* \param ticks # of ticks to delay
*/ 
void delayTicks(int ticks) { mpTaskDelay(ticks);};

 /**
  * \brief Utility function for writing a job file in temporary DRAM (the memory
  * supported by all controllers)
  *
  * \param path full path and name of file to create
  * \param job full job string
  *
  * \return true if file successfully opened
  */
 bool writeJob(char* path, char* job);
 
 /**
  * \brief Utility function for loading a job file from temporary DRAM (the memory
  * supported by all controllers)  WARNING: This function is limited to the DRAM
  * root directory.
  *
  * \param path path of the file to load (pass "" if the root directory is used)
  * \param job name of file to load (may not work with a full path)
  *
  * \return true if job successfully loaded
  */
 bool loadJob(char* path, char * job);
 
 /**
  * \brief Returns the velocity limit set for the controller.  This is stored within
  * the integer data table at VELOCITY_LIMIT_INDEX
  *
  * \return velocity limit (%)
  */
 double getVelocityLimit()
    {return (double) this->getInteger(VELOCITY_LIMIT_INDEX);};


  /**
  * \brief Reads the number of robot axes from the controller's config parameters.
  *
  * \param ctrl_grp Robot Control Group to retrieve. Zero-based (i.e. 0 = Control Group 1)
  * \param numAxes Number of robot axes (return)
  *
  * \return true if parameters successfully read
  */
  static bool getNumRobotAxes(int ctrl_grp, int* numAxes);
  static bool getNumRobotAxes(int* numAxes) { return getNumRobotAxes(active_ctrl_grp_, numAxes); }

 /**
  * \brief Reads the pulse-per-radian scaling factors from the controller's
  * config parameters, based on the arm's gearing ratios.
  *    jntPosInRadians = jntPosInPulses * pulseToRadian
  *
  * \param[in] ctrl_grp Robot Control Group to retrieve. Zero-based (i.e. 0 = Control Group 1)
  * \param[out] pulse_to_radian array of scaling factors (in Motoman order, length=MAX_PULSE_AXES)
  *
  * \return true if parameters successfully read
  */
 static bool getPulsesPerRadian(int ctrl_grp, float* pulses_per_radian);
 static bool getPulsesPerRadian(float* pulses_per_radian) { return getPulsesPerRadian(active_ctrl_grp_, pulses_per_radian); }

 /**
  * \brief Reads the pulse correction factors from the controller's
  * config parameters, based on physical axis cross-coupling.
  *    pulsePos[ulCorrectionAxis] -= pulsePos[ulSourceAxis] * fCorrectionRatio
  *
  * \param[in] ctrl_grp Robot Control Group to retrieve. Zero-based (i.e. 0 = Control Group 1)
  * \param[out] pulse_correction array of correction factors (length=MAX_PULSE_AXES)
  *
  * \return true if parameters successfully read
  */
 static bool getFBPulseCorrection(int ctrl_grp, FB_AXIS_CORRECTION* pulse_correction);
 static bool getFBPulseCorrection(FB_AXIS_CORRECTION* pulse_correction) { return getFBPulseCorrection(active_ctrl_grp_, pulse_correction); }
 
 /**
  * \brief Reads the number of allowed tasks from the controller's config parameters.
  *
  * \param[out] num_normal_tasks number of normal-priority tasks
  * \param[out] num_highPrio_tasks number of high-priority tasks
  * \param[out] num_out_files number of output files (??)
  *
  * \return true if parameters successfully read
  */
 static bool getNumTasks(int* num_normal_tasks, int* num_highPrio_tasks, int* num_out_files);

 /**
  * \brief Reads the number of milliseconds between each tick of the interpolation clock
  *   from the controller's config parameters.
  *
  * \param[out] interp_period period between interpolation ticks (msec)
  *
  * \return true if parameters successfully read
  */
 static bool getInterpPeriod(UINT16* interp_period);

 /**
  * \brief Reads the max increment each joint can move per interpolation cycle
  *   from the controller's config parameters.
  *
  * \param[in] ctrl_grp Robot Control Group to retrieve. Zero-based (i.e. 0 = Control Group 1)
  * \param[out] max_incr array of maximum increment values for each joint (length=MAX_PULSE_AXES)
  *
  * \return true if parameters successfully read
  */
 static bool getMaxIncrPerCycle(int ctrl_grp, int* max_incr);
 static bool getMaxIncrPerCycle(int* max_incr) { return getMaxIncrPerCycle(active_ctrl_grp_, max_incr); }

 /**
  * \brief Reads the maximum percentage (of maxIncrPerCycle) allowed for commanded motion
  *   from the controller's config parameters.
  *
  * \param[in] ctrl_grp Robot Control Group to retrieve. Zero-based (i.e. 0 = Control Group 1)
  * \param[out] limit percentage of MaxIncrPerCycle allowed for commanded motion
  *
  * \return true if parameters successfully read
  */
 static bool getIncrMotionLimit(int ctrl_grp, float* limit);
 static bool getIncrMotionLimit(float* limit) { return getIncrMotionLimit(active_ctrl_grp_, limit); }

 static int getCtrlGroup() { return active_ctrl_grp_; }
 static void setCtrlGroup(int ctrl_grp) { active_ctrl_grp_ = ctrl_grp; }

protected:

/**
* \brief Index within integer data table that holds velocity limit
*/
static const int VELOCITY_LIMIT_INDEX = 94;
	        
 /**
  * \brief Typical MP function call return on error
  */
 static const int MP_ERROR = -1;
 
 /**
  * \brief Typical MP function call return on OK (sometimes a value
  * greater than zero is also returned if it has some meaning, like
  * a file descriptor.
  */
 static const int MP_OK = 0;

  /**
  * \brief Poll delay (in ticks) when querying the motoplus api.
  */
 static const int VAR_POLL_DELAY_ = 10; //ms
 static const int UNIV_IN_DATA_START_ = 10;
 static const int UNIV_OUT_DATA_START_ = 10010;
 static const int UNIV_IO_DATA_SIZE_ = 2048;
 
 // Servo power variables
MP_SERVO_POWER_SEND_DATA servo_power_data;
MP_STD_RSP_DATA servo_power_error;

// Job variables
MP_START_JOB_SEND_DATA job_start_data;
MP_DELETE_JOB_SEND_DATA job_delete_data;
MP_STD_RSP_DATA job_error;


// Hold variables
MP_HOLD_SEND_DATA hold_data;
MP_STD_RSP_DATA hold_error;
 
//TODO: motion and job flags are just internal state variables, we may
//want to make them query the appropriate motoplus API calls instead.
/**
  * \brief True if motion enabled
  */
bool motionEnabled;
 
/**
  * \brief True if job started
  */
bool jobStarted;

/**
  * \brief Gets the current in motion status from the controller.  The
  * funciton queries the actual joint speeds so it should be independent
  * of any program state (which should be more reliable).
  *
  * \return motion status
  */
 industrial::robot_status::TriState getInMotionStatus();
 
static int active_ctrl_grp_;
 
static bool is_valid_ctrl_grp(int ctrl_grp);
static bool is_bit_set(int i, param_func_t type) { return (i & type); }
static void set_bit(int* i, param_func_t type) { *i |= type; }
};



} //controller
} //motoman

#endif //CONTROLLER_H