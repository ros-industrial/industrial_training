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

#include <console_bridge/console.h>

#include "descartes_moveit/moveit_state_adapter.h"
#include "descartes_core/pretty_print.hpp"
#include "descartes_moveit/seed_search.h"

#include <eigen_conversions/eigen_msg.h>
#include <random_numbers/random_numbers.h>
#include <ros/assert.h>
#include <sstream>

const static int SAMPLE_ITERATIONS = 10;

namespace
{
bool getJointVelocityLimits(const moveit::core::RobotState& state, const std::string& group_name,
                            std::vector<double>& output)
{
  std::vector<double> result;

  auto models = state.getJointModelGroup(group_name)->getActiveJointModels();
  for (const moveit::core::JointModel* model : models)
  {
    const auto& bounds = model->getVariableBounds();
    // Check to see if there is a single bounds constraint (more might indicate
    // not revolute joint)
    if (model->getType() != moveit::core::JointModel::REVOLUTE &&
        model->getType() != moveit::core::JointModel::PRISMATIC)
    {
      ROS_ERROR_STREAM(__FUNCTION__ << " Unexpected joint type. Currently works only"
                                       " with single axis prismatic or revolute joints.");
      return false;
    }
    else
    {
      result.push_back(bounds[0].max_velocity_);
    }
  }

  output = result;
  return true;
}

}  // end anon namespace

namespace descartes_moveit
{
MoveitStateAdapter::MoveitStateAdapter() : world_to_root_(Eigen::Affine3d::Identity())
{
}

bool MoveitStateAdapter::initialize(const std::string& robot_description, const std::string& group_name,
                                    const std::string& world_frame, const std::string& tcp_frame)
{
  // Initialize MoveIt state objects
  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader(robot_description));

  return initialize(robot_model_loader_->getModel(), group_name, world_frame, tcp_frame);
}

bool MoveitStateAdapter::initialize(robot_model::RobotModelConstPtr robot_model, const std::string &group_name,
                                    const std::string &world_frame, const std::string &tcp_frame)
{
  robot_model_ptr_ = robot_model;
  robot_state_.reset(new moveit::core::RobotState(robot_model_ptr_));
  robot_state_->setToDefaultValues();
  planning_scene_.reset(new planning_scene::PlanningScene(robot_model));
  joint_group_ = robot_model_ptr_->getJointModelGroup(group_name);

  // Assign robot frames
  group_name_ = group_name;
  tool_frame_ = tcp_frame;
  world_frame_ = world_frame;

  // Validate our model inputs w/ URDF
  if (!joint_group_)
  {
    logError("%s: Joint group '%s' does not exist in robot model", __FUNCTION__, group_name_.c_str());
    std::stringstream msg;
    msg << "Possible group names: " << robot_state_->getRobotModel()->getJointModelGroupNames();
    logError(msg.str().c_str());
    return false;
  }

  const auto& link_names = joint_group_->getLinkModelNames();
  if (tool_frame_ != link_names.back())
  {
    logWarn("%s: Tool frame '%s' does not match group tool frame '%s', functionality"
             "will be implemented in the future",
             __FUNCTION__, tool_frame_.c_str(), link_names.back().c_str());
  }

  if (!getJointVelocityLimits(*robot_state_, group_name, velocity_limits_))
  {
    logWarn("%s: Could not determine velocity limits of RobotModel from MoveIt", __FUNCTION__);
  }

  if (seed_states_.empty())
  {
    seed_states_ = seed::findRandomSeeds(*robot_state_, group_name_, SAMPLE_ITERATIONS);
    logDebug("Generated %lu random seeds", static_cast<unsigned long>(seed_states_.size()));
  }

  auto model_frame = robot_state_->getRobotModel()->getModelFrame();
  if (world_frame_ != model_frame)
  {
    logInform("%s: World frame '%s' does not match model root frame '%s', all poses will be"
              " transformed to world frame '%s'",
              __FUNCTION__, world_frame_.c_str(), model_frame.c_str(), world_frame_.c_str());

    Eigen::Affine3d root_to_world = robot_state_->getFrameTransform(world_frame_);
    world_to_root_ = descartes_core::Frame(root_to_world.inverse());
  }

  return true;
}

bool MoveitStateAdapter::getIK(const Eigen::Affine3d& pose, const std::vector<double>& seed_state,
                               std::vector<double>& joint_pose) const
{
  robot_state_->setJointGroupPositions(group_name_, seed_state);
  return getIK(pose, joint_pose);
}

bool MoveitStateAdapter::getIK(const Eigen::Affine3d& pose, std::vector<double>& joint_pose) const
{
  bool rtn = false;

  // transform to group base
  Eigen::Affine3d tool_pose = world_to_root_.frame * pose;

  if (robot_state_->setFromIK(joint_group_, tool_pose, tool_frame_))
  {
    robot_state_->copyJointGroupPositions(group_name_, joint_pose);
    if (!isValid(joint_pose))
    {
      ROS_DEBUG_STREAM("Robot joint pose is invalid");
    }
    else
    {
      rtn = true;
    }
  }
  else
  {
    rtn = false;
  }

  return rtn;
}

bool MoveitStateAdapter::getAllIK(const Eigen::Affine3d& pose, std::vector<std::vector<double> >& joint_poses) const
{
  // The minimum difference between solutions should be greater than the search discretization
  // used by the IK solver.  This value is multiplied by 4 to remove any chance that a solution
  // in the middle of a discretization step could be double counted.  In reality, we'd like solutions
  // to be further apart than this.
  double epsilon = 4 * joint_group_->getSolverInstance()->getSearchDiscretization();
  logDebug("Utilizing an min. difference of %f between IK solutions", epsilon);
  joint_poses.clear();
  for (size_t sample_iter = 0; sample_iter < seed_states_.size(); ++sample_iter)
  {
    robot_state_->setJointGroupPositions(group_name_, seed_states_[sample_iter]);
    std::vector<double> joint_pose;
    if (getIK(pose, joint_pose))
    {
      if (joint_poses.empty())
      {
        std::stringstream msg;
        msg << "Found *first* solution on " << sample_iter << " iteration, joint: " << joint_pose;
        logDebug(msg.str().c_str());
        joint_poses.push_back(joint_pose);
      }
      else
      {
        std::stringstream msg;
        msg << "Found *potential* solution on " << sample_iter << " iteration, joint: " << joint_pose;
        logDebug(msg.str().c_str());

        std::vector<std::vector<double> >::iterator joint_pose_it;
        bool match_found = false;
        for (joint_pose_it = joint_poses.begin(); joint_pose_it != joint_poses.end(); ++joint_pose_it)
        {
          if (descartes_core::utils::equal(joint_pose, (*joint_pose_it), epsilon))
          {
            logDebug("Found matching, potential solution is not new");
            match_found = true;
            break;
          }
        }
        if (!match_found)
        {
          std::stringstream msg;
          msg << "Found *new* solution on " << sample_iter << " iteration, joint: " << joint_pose;
          logDebug(msg.str().c_str());
          joint_poses.push_back(joint_pose);
        }
      }
    }
  }

  logDebug("Found %lu joint solutions out of %lu iterations", static_cast<unsigned long>(joint_poses.size()),
           static_cast<unsigned long>(seed_states_.size()));

  if (joint_poses.empty())
  {
    logError("Found 0 joint solutions out of %lu iterations", static_cast<unsigned long>(seed_states_.size()));
    return false;
  }
  else
  {
    logInform("Found %lu joint solutions out of %lu iterations", static_cast<unsigned long>(joint_poses.size()),
              static_cast<unsigned long>(seed_states_.size()));
    return true;
  }
}

bool MoveitStateAdapter::isInCollision(const std::vector<double>& joint_pose) const
{
  bool in_collision = false;
  if (check_collisions_)
  {
    robot_state_->setJointGroupPositions(group_name_, joint_pose);
    in_collision = planning_scene_->isStateColliding(*robot_state_, group_name_);
  }
  return in_collision;
}

bool MoveitStateAdapter::getFK(const std::vector<double>& joint_pose, Eigen::Affine3d& pose) const
{
  bool rtn = false;
  robot_state_->setJointGroupPositions(group_name_, joint_pose);
  if (isValid(joint_pose))
  {
    if (robot_state_->knowsFrameTransform(tool_frame_))
    {
      pose = world_to_root_.frame * robot_state_->getFrameTransform(tool_frame_);
      rtn = true;
    }
    else
    {
      logError("Robot state does not recognize tool frame: %s", tool_frame_.c_str());
      rtn = false;
    }
  }
  else
  {
    logError("Invalid joint pose passed to get forward kinematics");
    rtn = false;
  }

  return rtn;
}

bool MoveitStateAdapter::isValid(const std::vector<double>& joint_pose) const
{
  // Logical check on input sizes
  if (joint_group_->getActiveJointModels().size() != joint_pose.size())
  {
    logError("Size of joint pose: %lu doesn't match robot state variable size: %lu",
             static_cast<unsigned long>(joint_pose.size()),
             static_cast<unsigned long>(joint_group_->getActiveJointModels().size()));
    return false;
  }

  // Satisfies joint positional bounds?
  if (!joint_group_->satisfiesPositionBounds(joint_pose.data()))
  {
    return false;
  }
  // Is in collision (if collision is active)
  return !isInCollision(joint_pose);
}

bool MoveitStateAdapter::isValid(const Eigen::Affine3d& pose) const
{
  // TODO: Could check robot extents first as a quick check
  std::vector<double> dummy;
  return getIK(pose, dummy);
}

int MoveitStateAdapter::getDOF() const
{
  return joint_group_->getVariableCount();
}

bool MoveitStateAdapter::isValidMove(const std::vector<double>& from_joint_pose,
                                     const std::vector<double>& to_joint_pose, double dt) const
{
  // Check for equal sized arrays
  if (from_joint_pose.size() != to_joint_pose.size())
  {
    logError("To and From joint poses are of different sizes.");
    return false;
  }

  for (std::size_t i = 0; i < from_joint_pose.size(); ++i)
  {
    double dtheta = std::abs(from_joint_pose[i] - to_joint_pose[i]);
    double max_dtheta = dt * velocity_limits_[i];
    if (dtheta > max_dtheta)
      return false;
  }

  return true;
}

void MoveitStateAdapter::setState(const moveit::core::RobotState& state)
{
  ROS_ASSERT_MSG(static_cast<bool>(robot_state_), "'robot_state_' member pointer is null. Have you called "
                                                  "initialize()?");
  *robot_state_ = state;
  planning_scene_->setCurrentState(state);
}

}  // descartes_moveit
