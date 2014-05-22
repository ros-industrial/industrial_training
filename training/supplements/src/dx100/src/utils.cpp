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

#include <dx100/utils.h>
#include <dx100/definitions.h>
#include "ros/ros.h"
#include "urdf/model.h"
#include "math.h"

using namespace trajectory_msgs;
using namespace std;

namespace motoman
{
namespace utils
{

bool checkTrajectory(const trajectory_msgs::JointTrajectoryConstPtr& trajectory)
  {
    bool rtn = false;
    rtn = checkJointNames(trajectory);
    if (!rtn)
    {
      ROS_WARN("Joint trajectory name check failed");
    }
    return rtn;
  }


  bool checkJointNames(const JointTrajectoryConstPtr& trajectory)
  {
    bool rtn = false;

    // The maximum number of joints in the motoman controller is fixed
    if ((int)trajectory->joint_names.size() <= motoman::parameters::Parameters::JOINT_SUFFIXES_SIZE)
    {
      rtn = true;
    }
    else
    {
      rtn = false;
      ROS_WARN("Size of joint names: %zd, exceeds motoman size: %d",
               trajectory->joint_names.size(),
               motoman::parameters::Parameters::JOINT_SUFFIXES_SIZE);
    }

    if (rtn)
    {
      for(unsigned int i = 0; i<trajectory->joint_names.size(); i++)
      {
        string suffix(motoman::parameters::Parameters::JOINT_SUFFIXES[i]);
        if ( hasSuffix(trajectory->joint_names[i], suffix ) )
        {
          rtn = true;
        }
        else
        {
          rtn = false;
          break;
        }

      }
    }

    return rtn;
  }


  bool hasSuffix(const string &str, const string &suffix)
  {
    bool rtn = false;
    int result;

    // If an empty suffix is passed to the function it returns true (which is
    // to be expected).  This warning as a little help when this occurs.
    if (0 == suffix.size() )
    {
      ROS_WARN("Suffix of size zero passed to hasSuffix, continuing");
    }
    result = str.rfind(suffix);

    ROS_DEBUG("Checking string: %s for suffix: %s, result %d", str.c_str(), suffix.c_str(), result);
    if ( result == str.size()-suffix.size())
    {
      rtn = true;
      ROS_DEBUG("%s has the suffix: %s", str.c_str(), suffix.c_str());
    }
    else
    {
      rtn = false;
      ROS_DEBUG("%s does not have the suffix: %s", str.c_str(), suffix.c_str());
    }

    return rtn;
  }


  bool getVelocityLimits(std::string param_name,
                         JointTrajectoryConstPtr trajectory,
                         std::vector<double> &joint_velocity_limits)
  {
    bool rtn = false;
    string urdf;

    joint_velocity_limits.clear();
    joint_velocity_limits.resize(trajectory->joint_names.size(), 0.0);

    if (ros::param::get(param_name, urdf))
    {
      urdf::Model urdf_model;
      if (urdf_model.initString(urdf))
      {

        for(unsigned int i = 0; i<trajectory->joint_names.size(); i++)
        {
          double limit;
          boost::shared_ptr<const urdf::Joint> joint = urdf_model.getJoint(trajectory->joint_names[i]);
          limit = joint->limits->velocity;
          joint_velocity_limits[i] = limit;
          ROS_DEBUG("Found joint velocity limit: %e for joint: %s", 
                    joint_velocity_limits[i], trajectory->joint_names[i].c_str());
        }
        ROS_DEBUG("Successefully populated velocity limits, size: %d", joint_velocity_limits.size());
        rtn = true;
      }
      else
      {
        ROS_ERROR("Failed to parse urdf xml string");
      }
    }
    else
    {
      ROS_ERROR("Failed to get urdf from parameter: %s", param_name.c_str());
      rtn = false;
    }

    if (!rtn)
    {
      ROS_ERROR("Failed to get velocity limits for parameter: %s", param_name.c_str());
    }

    return rtn;



  }


  double toMotomanVelocity(std::vector<double> &joint_velocity_limits,
                           std::vector<double> &joint_velocities)
  {
    double maxVelPct = 0.0;

    ROS_DEBUG("Converting to motoman velocity, limit size: %d, velocity size: %d",
              joint_velocity_limits.size(), joint_velocities.size());
    if (joint_velocity_limits.size() == joint_velocities.size())
    {
      for(int i = 0; i<joint_velocity_limits.size(); i++)
      {
        if (joint_velocity_limits[i] > 0.0)
        {
          ROS_DEBUG("Calculating velocity percent");
          double velPct = joint_velocities[i]/joint_velocity_limits[i];

          ROS_DEBUG("Calculating velocity percent, velocity: %e, limit: %e, percent: %e",
                    joint_velocities[i], joint_velocity_limits[i], velPct);
          if (abs(velPct) > maxVelPct)
          {
            ROS_DEBUG("Calculated velocity: %e, greater than current max: %e",
                      velPct, maxVelPct);
            maxVelPct = abs(velPct);
          }
        }
        else
        {
          ROS_ERROR("Invalid joint velocity: %e or limit: %e", joint_velocities[i],
                    joint_velocity_limits[i]);
        }
      }
    }
    else
    {
      ROS_ERROR("Failed to calculate a velocity (joint velocity and limit vectors different size");
    }


    if (maxVelPct > 1.0)
    {
      ROS_WARN("Max velocity percent: %e, exceed 1.0, setting to 1.0", maxVelPct);
      maxVelPct = 1.0;
    }
    else if (maxVelPct < 0.0)
    {
      ROS_WARN("Max velocity percent: %e, is less than 0.0, setting to 0.0", maxVelPct);
      maxVelPct = 0.0;
    }

    ROS_DEBUG("Returning a motoman velocity of: %e", maxVelPct);

    return maxVelPct;
  }


} //utils
} //motoman
