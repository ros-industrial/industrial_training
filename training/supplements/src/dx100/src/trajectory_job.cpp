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
#ifdef ROS
#include "dx100/ros_conversion.h"
#include "dx100/trajectory_job.h"
#include "string.h"
#include "simple_message/joint_traj_pt.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"
#define MAX_PULSE_AXES (8)
#endif

#ifdef MOTOPLUS
#include "ros_conversion.h"
#include "trajectory_job.h"
#include "string.h"
#include "joint_traj_pt.h"
#include "shared_types.h"
#include "log_wrapper.h"
#include "motoPlus.h"
#endif

using namespace industrial::shared_types;
using namespace industrial::joint_traj;
using namespace industrial::joint_traj_pt;
using namespace industrial::joint_data;
using namespace motoman::ros_conversion;

namespace motoman
{
namespace trajectory_job
{

/**
 * \brief Performs a safe string concat. by checking the destination size before
 * performing the string concat.  A function return of "false" occurs if the desination
 * is not large enough to receive the src string.  A macro is used so that the function
 * returns immediately from the calling function.  This cleans up the code and does
 * not result in lots of if/else statements.
 */

// A character is reserved for NULL (not sure it is needed)

#define SAFE_STRCAT(dest, dest_max_size, src) \
do \
{ \
  if (strlen(src) < dest_max_size - strlen(dest) - 1) \
  { \
    strcat(dest, src); \
  } \
  else \
  { \
    LOG_ERROR("STRCAT failed to append %s to %s", src, dest); \
    return false; \
  } \
} while (0)

/**
 * \brief Performs a safe line feed and carriage return appending using STRCAT.  See 
 * STRCAT for explanation of macro use
 */

#define APPEND_LINEFEED(dest, dest_max_size) SAFE_STRCAT(dest, dest_max_size, "\r\n");

/**
 * \brief Performs a safe line append using STRCAT and APPEND_LINE.  See 
 * STRCAT for explanation of macro use
 */

#define APPEND_LINE(dest, dest_max_size, src) \
do \
{ \
  SAFE_STRCAT(dest, dest_max_size, src); \
  APPEND_LINEFEED(dest, dest_max_size); \
} while (0)

TrajectoryJob::TrajectoryJob(void)
{
}
TrajectoryJob::~TrajectoryJob(void)
{
}

bool TrajectoryJob::init(const char* name)
{

  bool rtn = false;

  if( strlen(name) <= NAME_BUFFER_SIZE_ )
  {
    strcpy(this->name_, name);
    rtn = true;
  }
  else
  {
    LOG_ERROR("Failed to initialize trajectory job, name size %d too large",
              NAME_BUFFER_SIZE_);
    rtn = false;
  }
  return rtn;

}

bool TrajectoryJob::toJobString(JointTraj & trajectory, char* str_buffer, size_t buffer_size)
{
  bool rtn = false;
  if (0 < buffer_size)
  {

    // Header begin
    // ----------------------------------------------------------------------------
    strcpy(str_buffer, "");
    APPEND_LINE(str_buffer, buffer_size, "/JOB");

    sprintf(this->line_buffer_, "//NAME %s", this->name_);
    APPEND_LINE(str_buffer, buffer_size, this->line_buffer_);

    APPEND_LINE(str_buffer, buffer_size, "//POS");

    sprintf(this->line_buffer_, "///NPOS %d,0,0,0,0,0", trajectory.size());
    APPEND_LINE(str_buffer, buffer_size, this->line_buffer_);

    APPEND_LINE(str_buffer, buffer_size, "///TOOL 0");
    APPEND_LINE(str_buffer, buffer_size, "///POSTYPE PULSE");
    APPEND_LINE(str_buffer, buffer_size, "///PULSE");

    // Point declaration and initialization
    // ----------------------------------------------------------------------------
    JointTrajPt pt;
    JointData rosData;
    float mpRadians[MAX_PULSE_AXES];
    long mpPulses[MAX_PULSE_AXES];

    for(int i = 0; i < trajectory.size(); i++)
    {
      sprintf(this->line_buffer_, "C%05d=", i);
      SAFE_STRCAT(str_buffer, buffer_size, this->line_buffer_);
      trajectory.getPoint(i, pt);
      pt.getJointPosition(rosData);
      
      // Converting to a motoplus joint (i.e. the correct order and units
      toMpJoint(rosData, mpRadians);
      memset(mpPulses, 0, MAX_PULSE_AXES);  // can't convert radian->pulse on PC-side
      LOG_ERROR("Failed to create Job string: Radian->Pulse scaling is not available for PC-side code.");
      return false;

      for(int j = 0; j < MAX_PULSE_AXES; j++)
      {
        // Don't append comma to last position, instead line-feed.
        if (j < (MAX_PULSE_AXES - 1))
        {
          sprintf(this->line_buffer_, "%d,", mpPulses[j]);
          SAFE_STRCAT(str_buffer, buffer_size, this->line_buffer_);
        }
        else
        {
          sprintf(this->line_buffer_, "%d", mpPulses[j]);
          APPEND_LINE(str_buffer, buffer_size, this->line_buffer_);
        }
      }
    }

    // Header end
    // ----------------------------------------------------------------------------
    APPEND_LINE(str_buffer, buffer_size, "//INST");
    APPEND_LINE(str_buffer, buffer_size, "///DATE 1979/10/01 00:00");
    APPEND_LINE(str_buffer, buffer_size, "///ATTR SC,RW");
    APPEND_LINE(str_buffer, buffer_size, "///GROUP1 RB1");

    // Program
    APPEND_LINE(str_buffer, buffer_size, "NOP");
    for(int i = 0; i < trajectory.size(); i++)
    {
      trajectory.getPoint(i, pt);
      sprintf(this->line_buffer_, "MOVJ C%05d VJ=%.2f", i, pt.getVelocity());
      APPEND_LINE(str_buffer, buffer_size, this->line_buffer_);
    }
    APPEND_LINE(str_buffer, buffer_size, "END");

    rtn = true;
  }
  else
  {
    rtn = false;
    LOG_ERROR("Failed to generate job string");
  }

  return rtn;

}


}
}

