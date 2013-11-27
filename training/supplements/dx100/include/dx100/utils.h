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

#ifndef _MOTOMAN_UTILS_H
#define _MOTOMAN_UTILS_H

#include <vector>
#include <string>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>


namespace motoman
{
namespace utils
{

  /**
 * \brief Checks that the trajectory meets the assumptions and requirements
 * of the motoman controller interface.
 *
 * \param trajectory to check
 *
 * \return True if trajectory is valid and meets requirements
 */
bool checkTrajectory(const trajectory_msgs::JointTrajectoryConstPtr& trajectory);
bool checkTrajectory();

  /**
 * \brief Checks that the joint names match the assumptions made by the motoman
 * controller interface.
 *
 * \param trajectory with joint names to check
 *
 * \return True if joint names and order match the expected motoman order
 */
  bool checkJointNames(const trajectory_msgs::JointTrajectoryConstPtr& trajectory);

  /**
  * \brief Checks a string a the suffix
  *
  * \param string to check
  * \param suffix to check for
  *
  * \return True if str has the suffix
  */
   bool hasSuffix(const std::string &str, const std::string &suffix);

  /**
  * \brief Queries the URDF parameter server to determine what the velocity limits
  * are for the joint names that are pased in.  The order of the velocity limits
  * returned (by reference) matches the joint name order.  NOTE: This process is
  * an expensive one.  The results of this call should be cached for future needs.
  *
  * \param parameter name to query (typically \robot_description)
  * \param trajectory with joint names to query
  * \param joint velocity limits returned by URDQ query
  *
  * \return True if all velocities were found
  */
   bool getVelocityLimits(std::string param_name,
                          trajectory_msgs::JointTrajectoryConstPtr trajectory,
                          std::vector<double> &joint_velocity_limits);

  /**
 * \brief Converts the ROS trajectory velocity (individual joint velocities to
 * the combined motoman velocity which is highest joint velocity required as a
 * fraction of the joint velocity limit.
 *
 * \param joint velocity limits (order should match joint velocities)
 * \param joint velocities (order should match limits)
 *
 * \return Combined velocity
 */
  double toMotomanVelocity(std::vector<double> &joint_velocity_limits,
                         std::vector<double> &joint_velocities);

} // utils
} // motoman

#endif
