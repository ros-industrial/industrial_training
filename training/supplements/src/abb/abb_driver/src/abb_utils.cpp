/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Southwest Research Institute
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
 *       * Neither the name of the Southwest Research Institute, nor the names
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

#include "abb_driver/abb_utils.h"
#include "ros/ros.h"

namespace abb
{
namespace utils
{

// TBD: This transform should also account for velocity/acceleration affects due to linkage, so that velocity calculation is accurate
void linkage_transform(const trajectory_msgs::JointTrajectoryPoint& pt_in, trajectory_msgs::JointTrajectoryPoint* pt_out, double J23_factor)
{
  *pt_out = pt_in;
  linkage_transform(pt_in.positions, &(pt_out->positions), J23_factor);
}

void linkage_transform(const std::vector<double>& points_in, std::vector<double>* points_out, double J23_factor)
{
  ROS_ASSERT(points_in.size() > 3);

  *points_out = points_in;
  points_out->at(2) += J23_factor * points_out->at(1);
}

} //abb

} //utils
