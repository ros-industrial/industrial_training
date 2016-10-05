/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2016, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/*
 *  ros_conversions.cpp
 *
 *  Created on: Feb 28, 2016
 *  Author: Jonathan Meyer
 */

#include "descartes_utilities/ros_conversions.h"
#include <algorithm>
#include <console_bridge/console.h>

/**
 * @brief Given two sets of joint values representing two robot joint poses, this function computes the
 *        minimum amount of time required to interpolate between the points assuming that each joint can
 *        move no faster than a constant 'max_vel' speed.
 * @return The minimum time required to interpolate between poses.
 */
static double minTime(const std::vector<double>& pose_a, const std::vector<double>& pose_b, double max_vel)
{
  std::vector<double> diff;
  diff.reserve(pose_a.size());

  // compute joint-wise minimum time required based on relative distance between joint positions
  // and the maximum allowable joint velocity
  std::transform(pose_a.begin(), pose_a.end(), pose_b.begin(), std::back_inserter(diff), [max_vel](double a, double b)
                 {
                   return std::abs(a - b) / max_vel;
                 });
  // The biggest time across all of the joints is the min time
  return *std::max_element(diff.begin(), diff.end());
}

bool descartes_utilities::toRosJointPoints(const descartes_core::RobotModel& model,
                                           const std::vector<descartes_core::TrajectoryPtPtr>& joint_traj,
                                           double default_joint_vel,
                                           std::vector<trajectory_msgs::JointTrajectoryPoint>& out)
{
  if (default_joint_vel <= 0.0)
  {
    logError("%s: Invalid value for default joint velocity. Must be > 0 (radians/second)", __FUNCTION__);
    return false;
  }

  const static double max_default_joint_velocity = 100.0;  // (radians / s); approx 1000 rpm
  if (default_joint_vel > max_default_joint_velocity)
  {
    logError("%s: Default joint velocity of %f exceeds assumed limit of %f.", __FUNCTION__, default_joint_vel,
             max_default_joint_velocity);
    return false;
  }

  ros::Duration from_start(0.0);
  std::vector<trajectory_msgs::JointTrajectoryPoint> ros_trajectory;
  ros_trajectory.reserve(joint_traj.size());

  std::vector<double> joint_point;
  std::vector<double> dummy;

  for (std::size_t i = 0; i < joint_traj.size(); ++i)
  {
    if (!joint_traj[i])
    {
      logError("%s: Input trajectory contained null pointer at index %lu", __FUNCTION__, static_cast<unsigned long>(i));
      return false;
    }

    descartes_core::TrajectoryPt& pt = *joint_traj[i];

    if (!pt.getNominalJointPose(dummy, model, joint_point))
    {
      logError("%s: Failed to extract joint positions from input trajectory at index %lu", __FUNCTION__,
               static_cast<unsigned long>(i));
      return false;
    }

    trajectory_msgs::JointTrajectoryPoint ros_pt;
    ros_pt.positions = joint_point;
    // Descartes has no internal representation of velocity, acceleration, or effort so we fill these field with zeros.
    ros_pt.velocities.resize(joint_point.size(), 0.0);
    ros_pt.accelerations.resize(joint_point.size(), 0.0);
    ros_pt.effort.resize(joint_point.size(), 0.0);

    if (pt.getTiming().isSpecified())
    {
      from_start += ros::Duration(pt.getTiming().upper);
    }
    else
    {
      // If we have a previous point, compute dt based on default, max joint velocity
      // otherwise trajectory starts at current location (time offset == 0).
      double dt;
      if (i == 0)
        dt = 0.0;
      else
        dt = minTime(joint_point, ros_trajectory.back().positions, default_joint_vel);

      from_start += ros::Duration(dt);
    }

    ros_pt.time_from_start = from_start;
    ros_trajectory.push_back(ros_pt);
  }

  out = ros_trajectory;
  return true;
}
