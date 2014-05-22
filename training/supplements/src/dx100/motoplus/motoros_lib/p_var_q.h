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

#ifndef __p_var_q_h
#define __p_var_q_h

#include "motoPlus.h"
#include "controller.h"
#include "joint_data.h"

/*
Point variable queue

The point variable queue class encapsulates the point variable buffer
used to buffer robot motion points.  The robot controller requires a
point buffer in order to do it's own path interpolation and 
ensure smooth motion between points.  This buffer exists between the
motoplus application and the INFORM motion program.  (An INFORM motion
program is used because it can perform smooth motion through several
points, motoplus cannot).  The following section describes the structure
and function of the point variable queue.

Variables:

Position variables (joint type)
(NOTE: The max number of P variables is limited by the controller)
P000 - PXXX(see QSIZE) units in pulses

Velocity variables (integer type)
When specifying a VJ speed in percentage, the controller calculates the 
time it takes for each motor to travel that its distance (in pulse) at 
specified speed. After it finds the motor that will take the longest time, 
it will adjust down the speed of all the other motors so they will take the 
same time.

I000 - IXXX(see QSIZE) (percent 0.01%-100% -> 1-10000)

Buffer Management
The following variables are used to manage the buffer.  Two variables are used.
The current Motion pointer is the index of the point currently being exectuted as
part of the MOVJ (NOTE: Depending on how the controller performs it's look ahead,
this may not be the point that it is currently executing motion on)  
The current buffer pointer is the last point was populated with a valid joint point.

NOTE: These variables are hard-codes in INFORM.  If their indexes change than the
INFORM program must also be updated.

IXXX(MOTION_POINTER) index of current point being executed in MOVJ
IXXX(BUFFER_POINTER) index of the last populated point.

Given these two variables the following can be determined.

BUFFER_SIZE = BUFFER_POINTER - MOTION_POINTER

POSITION VARIABLE INEXES:
MOTION_POS_INDEX        = MOTION_POINTER % QSIZE
BUFFER_POS_INDEX        = BUFFER_POINTER % QSIZE
NEXT_BUFFER_POS_INDEX   = (BUFFER_POINTER + 1) % QSIZE

MAX_BUFFER_SIZE <= QSIZE - 1 (or less, if desired)

The INFORM program should only execute MOVJ on those points when the
BUFFER_POINTER >=  MOTION_POINTER.

TODO: WHAT TO DO WITH INTEGER ROLL OVERS ON MOTION/BUFFER POINTERS.

*/


namespace motoman
{
namespace p_var_q
{

// Utility macro for some compile time checking.
#define STATIC_ASSERT(COND,MSG) typedef char static_assertion_##MSG[(COND)?1:-1]

using motoman::controller::Controller;

class PVarQ
// Holds data and functions for executing position variable queue motion
{
  public:
    PVarQ();
    ~PVarQ(void);
    
    
  /**
  * \brief Initializes the queue (sets the initial joint position)
  *
  * \param value to set joint position to (IN ROS JOINT ORDER)
  * \param percent velocity (will be converted to appropriate integer (see note for jointSpeedData_)
  *
  */
    void init(industrial::joint_data::JointData & point, double velocity_percent);
    
    /**
  * \brief Adds point to the queue (will block until point can be added)
  *
  * \param joint position to add
  */
    void addPoint(industrial::joint_data::JointData & joints, double velocity_percent);
    
      /**
  * \brief Return current buffer size (number of remaining points)  
  * Should be used to determine if more points can be added.
  *
  * \return current buffer size
  */
    int bufferSize();
    
    
          /**
  * \brief Return number of position variables available in the queue.
  * The number will be greater than or equal to the max buffer size + 1.
  *
  * \return number of position variables in queue
  */
    int posVarQueueSize() {return QSIZE_;};
    
    
        /**
  * \brief Return maximum buffer size (Queue size - 1)  The max buffer size
  * should be less than the QSIZE minus whatver the motoman look-ahead is.
  *
  * \return max buffer size
  */
    int maxBufferSize() {return (posVarQueueSize() - PT_LOOK_AHEAD_);};
    
    
        /**
  * \brief Return motion pointer index
  *
  * \return motion pointer index
  */
    int getMotionIndex()
    {return Controller::getInteger(MOTION_POINTER_);};

          /**
  * \brief Return buffer pointer index
  *
  * \return buffer pointer index
  */
    int getBufferIndex()
    {return Controller::getInteger(BUFFER_POINTER_);};
  
             /**
  * \brief Return motion position variable index
  *
  * \return motion position variable index
  */
    int getMotionPosIndex() ;
  
  
           /**
  * \brief Return buffer position variable index
  *
  * \return buffer position variable index
  */
    int getBufferPosIndex();
    
             /**
  * \brief Return next buffer position variable index
  *
  * \return next buffer position variable index
  */
    int getNextBufferPosIndex();
    
              /**
  * \brief Return true if buffer is full (points cannot be added)
  *
  * \return true if buffer is full
  */
    bool bufferFull();
    
        
              /**
  * \brief Return true if buffer is empty
  *
  * \return true if buffer is empty
  */
    bool bufferEmpty();
    
    
               /**
  * \brief Get temporary velocity variable (Will be removed in the future)
  *
  * \return velocity percent
  */
    double TEMP_getVelocityPercent()
    {return (double) Controller::getInteger(TEMP_VELOCITY_POINTER);};
  
  
		
  protected:

	static const int VAR_POLL_TICK_DELAY_ = 10;
	
   /**
   * \brief number of ticks to delay between buffer polling
   */
	static const int BUFFER_POLL_TICK_DELAY_ = 100;
	
	 /**
   * \brief Size of the INFORM motion queue (i.e. the number of
   * position variables in the MOTION_JOB_NAME_.
   */
    static const int QSIZE_ = 20;
    
     /**
   * \brief Number of points the INFORM motion planner looks
   * ahead.
   */
    static const int PT_LOOK_AHEAD_ = 5;
    
     /**
   * \brief Integer index in INFORM integer variable table where
   * motion pointer is stored (INDEX MUST BE GREATER THAN QSIZE,
   * INTEGERS ALIGNED WITH POSITION VARIABLES ARE USED TO SET
   * MOTION SPEED)
   */
    static const int MOTION_POINTER_ = 91;
    
      /**
   * \brief Integer index in INFORM integer variable table where
   * buffer pointer is stored (INDEX MUST BE GREATER THAN QSIZE,
   * INTEGERS ALIGNED WITH POSITION VARIABLES ARE USED TO SET
   * MOTION SPEED)
   */
    static const int BUFFER_POINTER_ = 92;
    
    
      /**
   * \brief Integer index in INFORM integer variable table where the
   * minimum buffer size for starting motion is.  (INDEX MUST BE GREATER THAN QSIZE,
   * INTEGERS ALIGNED WITH POSITION VARIABLES ARE USED TO SET
   * MOTION SPEED)
   */
    static const int MIN_BUF_START_POINTER_ = 93;
    
       /**
   * \brief Temporary integer value for holding move velocity (WILL BE REMOVED
   * WHEN TRAJECTORY VELOCITY IS USED)
   */
    static const int TEMP_VELOCITY_POINTER = 94;
    
	
	        /**
  * \brief Increments buffer index
  *
  */
    void incBufferIndex();
   
  /**
  * \brief Set position variable that is next in the queue
  *
  * \param value to set joint position to (IN ROS JOINT ORDER)
  * \param percent velocity (will be converted to appropriate integer (see note for jointSpeedData_)
  *
  */
    void setNextPosition(industrial::joint_data::JointData & point, double velocity_percent);
    
  /**
  * \brief Set position variable in the queue position(index)
  *
  * \param position index
  * \param value to set joint position to (IN ROS JOINT ORDER)
  * \param percent velocity (will be converted to appropriate integer (see note for jointSpeedData_)
  *
  */
    void setPosition(int index, industrial::joint_data::JointData & point, 
    double velocity_percent);
			
};


}//namespace p_var_q
}//namespace motoman



#endif