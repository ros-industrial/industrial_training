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
 *  ros_conversions.h
 *
 *  Created on: Feb 28, 2016
 *  Author: Jonathan Meyer
 */

#ifndef DESCARTES_ROS_CONVERSIONS_H
#define DESCARTES_ROS_CONVERSIONS_H

#include <trajectory_msgs/JointTrajectory.h>
#include <descartes_core/trajectory_pt.h>

namespace descartes_utilities
{
/**
 * @brief Converts a sequence of Descartes joint trajectory points to ROS trajectory points.
 *        Copies timing if specified, and sets vel/acc/effort fields to zeros.
 * @param model Descartes robot model associated with the Descartes joint trajectory
 * @param joint_traj Sequence of 'joint trajectory points' as returned by Dense/Sparse planner
 * @param default_joint_vel If a point, does not have timing specified, this value (in rads/s)
 *                          is used to calculate a 'default' time. Must be > 0 & less than 100.
 * @param out Buffer in which to store the resulting ROS trajectory. Only overwritten on success.
 * @return True if the conversion succeeded. False otherwise.
 */
bool toRosJointPoints(const descartes_core::RobotModel& model,
                      const std::vector<descartes_core::TrajectoryPtPtr>& joint_traj, double default_joint_vel,
                      std::vector<trajectory_msgs::JointTrajectoryPoint>& out);
}

#endif
