/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, Dan Solomon
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
 * joint_trajectory_pt.cpp
 *
 *  Created on: Oct 3, 2014
 *      Author: Dan Solomon
 */

#include <console_bridge/console.h>
#include "descartes_trajectory/joint_trajectory_pt.h"

#define NOT_IMPLEMENTED_ERR(ret) logError("%s not implemented", __PRETTY_FUNCTION__); return ret;

using namespace descartes_core;
namespace descartes_trajectory
{

JointTrajectoryPt::JointTrajectoryPt():
    tool_(Eigen::Affine3d::Identity()),
    wobj_(Eigen::Affine3d::Identity())
{}

JointTrajectoryPt::JointTrajectoryPt(const std::vector<TolerancedJointValue> &joints,
                                     const Frame &tool, const Frame &wobj):
  joint_position_(joints),
  tool_(tool),
  wobj_(wobj)
{}

JointTrajectoryPt::JointTrajectoryPt(const std::vector<TolerancedJointValue> &joints):
  joint_position_(joints),
  tool_(Eigen::Affine3d::Identity()),
  wobj_(Eigen::Affine3d::Identity())
{}

JointTrajectoryPt::JointTrajectoryPt(const std::vector<double> &joints):
  tool_(Eigen::Affine3d::Identity()),
  wobj_(Eigen::Affine3d::Identity())
{
  for (size_t ii = 0; ii < joints.size(); ++ii)
  {
    joint_position_.push_back(TolerancedJointValue(joints[ii]));
  }
}


bool JointTrajectoryPt::getClosestCartPose(const std::vector<double> &seed_state,
                                           const RobotModel &model, Eigen::Affine3d &pose) const
{
  NOT_IMPLEMENTED_ERR(false)
}

bool JointTrajectoryPt::getNominalCartPose(const std::vector<double> &seed_state,
                                           const RobotModel &model, Eigen::Affine3d &pose) const
{
  std::vector<double> joints;
  for(auto& tj: joint_position_)
  {
    joints.push_back(tj.nominal);
  }
  return model.getFK(joints,pose);
}

void JointTrajectoryPt::getCartesianPoses(const RobotModel &model, EigenSTL::vector_Affine3d &poses) const
{
  poses.clear();
}

bool JointTrajectoryPt::getClosestJointPose(const std::vector<double> &seed_state,
                                            const RobotModel &model,
                                            std::vector<double> &joint_pose) const
{
  if(joint_position_.empty())
  {
    return false;
  }
  else
  {
    return getNominalJointPose(seed_state,model,joint_pose);
  }
}

bool JointTrajectoryPt::getNominalJointPose(const std::vector<double> &seed_state,
                                            const RobotModel &model,
                                            std::vector<double> &joint_pose) const
{
  joint_pose.resize(joint_position_.size());
  for (size_t ii=0; ii<joint_position_.size(); ++ii)
  {
    joint_pose[ii] = joint_position_[ii].nominal;
  }
  return true;
}

void JointTrajectoryPt::getJointPoses(const RobotModel &model,
                                      std::vector<std::vector<double> > &joint_poses) const
{
  std::vector<double> empty_seed;
  joint_poses.resize(1);
  getNominalJointPose(empty_seed,model,joint_poses[0]);
}

bool JointTrajectoryPt::isValid(const RobotModel &model) const
{
  std::vector<double> lower(joint_position_.size());
  std::vector<double> upper(joint_position_.size());
  for (size_t ii = 0; ii < joint_position_.size(); ++ii)
  {
    lower[ii] = joint_position_[ii].tolerance.lower;
    upper[ii] = joint_position_[ii].tolerance.upper;
  }
return model.isValid(lower) && model.isValid(upper);
}

bool JointTrajectoryPt::setDiscretization(const std::vector<double> &discretization)
{
  if (discretization.size() != 1 || discretization.size() != joint_position_.size())
  {
    logError("discretization must be size 1 or same size as joint count.");
    return false;
  }

  if (discretization.size() == 1)
  {
    discretization_ = std::vector<double>(joint_position_.size(), discretization[0]);
    return true;
  }

  /* Do not copy discretization values until all values are confirmed */
  for (size_t ii=0; ii<discretization.size(); ++ii)
  {
    if (discretization[ii] < 0. || discretization[ii] > joint_position_[ii].range())
    {
      logError("discretization value out of range.");
      return false;
    }
  }

  discretization_ = discretization;

  return true;
}

} /* namespace descartes_trajectory */
