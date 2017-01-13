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

#define NOT_IMPLEMENTED_ERR(ret)                                                                                       \
  logError("%s not implemented", __PRETTY_FUNCTION__);                                                                 \
  return ret;

// Utility function to unpack joint bounds from a TolerancedJointValue struct
// Note that this does not clear the existing vectors.
static void unpackTolerancedJoints(const std::vector<descartes_trajectory::TolerancedJointValue> &tolerances,
                                   std::vector<double> &lower, std::vector<double> &nominal, std::vector<double> &upper)
{
  lower.reserve(tolerances.size());
  nominal.reserve(tolerances.size());
  upper.reserve(tolerances.size());

  for (std::size_t i = 0; i < tolerances.size(); ++i)
  {
    lower.push_back(tolerances[i].lower);
    nominal.push_back(tolerances[i].nominal);
    upper.push_back(tolerances[i].upper);
  }
}

using namespace descartes_core;
namespace descartes_trajectory
{
JointTrajectoryPt::JointTrajectoryPt(const descartes_core::TimingConstraint &timing)
  : descartes_core::TrajectoryPt(timing), tool_(Eigen::Affine3d::Identity()), wobj_(Eigen::Affine3d::Identity())
{
}

JointTrajectoryPt::JointTrajectoryPt(const std::vector<TolerancedJointValue> &joints, const Frame &tool,
                                     const Frame &wobj, const descartes_core::TimingConstraint &timing)
  : descartes_core::TrajectoryPt(timing), tool_(tool), wobj_(wobj)
{
  unpackTolerancedJoints(joints, lower_, nominal_, upper_);
}

JointTrajectoryPt::JointTrajectoryPt(const std::vector<TolerancedJointValue> &joints,
                                     const descartes_core::TimingConstraint &timing)
  : descartes_core::TrajectoryPt(timing), tool_(Eigen::Affine3d::Identity()), wobj_(Eigen::Affine3d::Identity())
{
  unpackTolerancedJoints(joints, lower_, nominal_, upper_);
}

JointTrajectoryPt::JointTrajectoryPt(const std::vector<double> &joints, const descartes_core::TimingConstraint &timing)
  : nominal_(joints)
  , lower_(joints)
  , upper_(joints)
  , descartes_core::TrajectoryPt(timing)
  , tool_(Eigen::Affine3d::Identity())
  , wobj_(Eigen::Affine3d::Identity())
{
}

bool JointTrajectoryPt::getClosestCartPose(const std::vector<double> &seed_state, const RobotModel &model,
                                           Eigen::Affine3d &pose) const
{
  NOT_IMPLEMENTED_ERR(false)
}

bool JointTrajectoryPt::getNominalCartPose(const std::vector<double> &seed_state, const RobotModel &model,
                                           Eigen::Affine3d &pose) const
{
  return model.getFK(nominal_, pose);
}

void JointTrajectoryPt::getCartesianPoses(const RobotModel &model, EigenSTL::vector_Affine3d &poses) const
{
  poses.clear();
}

bool JointTrajectoryPt::getClosestJointPose(const std::vector<double> &seed_state, const RobotModel &model,
                                            std::vector<double> &joint_pose) const
{
  if (nominal_.empty())
  {
    return false;
  }
  else
  {
    return getNominalJointPose(seed_state, model, joint_pose);
  }
}

bool JointTrajectoryPt::getNominalJointPose(const std::vector<double> &seed_state, const RobotModel &model,
                                            std::vector<double> &joint_pose) const
{
  joint_pose.assign(nominal_.begin(), nominal_.end());
  return true;
}

void JointTrajectoryPt::getJointPoses(const RobotModel &model, std::vector<std::vector<double> > &joint_poses) const
{
  std::vector<double> empty_seed;
  joint_poses.resize(1);
  getNominalJointPose(empty_seed, model, joint_poses[0]);
}

bool JointTrajectoryPt::isValid(const RobotModel &model) const
{
  return model.isValid(lower_) && model.isValid(upper_);
}

bool JointTrajectoryPt::setDiscretization(const std::vector<double> &discretization)
{
  if (discretization.size() != 1 || discretization.size() != nominal_.size())
  {
    logError("discretization must be size 1 or same size as joint count.");
    return false;
  }

  if (discretization.size() == 1)
  {
    discretization_ = std::vector<double>(nominal_.size(), discretization[0]);
    return true;
  }

  /* Do not copy discretization values until all values are confirmed */
  for (size_t ii = 0; ii < discretization.size(); ++ii)
  {
    if (discretization[ii] < 0. || discretization[ii] > (upper_[ii] - lower_[ii]))
    {
      logError("discretization value out of range.");
      return false;
    }
  }

  discretization_ = discretization;

  return true;
}

void JointTrajectoryPt::setJoints(const std::vector<TolerancedJointValue> &joints)
{
  lower_.clear();
  nominal_.clear();
  upper_.clear();
  unpackTolerancedJoints(joints, lower_, nominal_, upper_);
}

} /* namespace descartes_trajectory */
