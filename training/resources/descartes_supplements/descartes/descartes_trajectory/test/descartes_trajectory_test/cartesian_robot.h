/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, Southwest Research Institute
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

#ifndef CARTESIAN_ROBOT_H_
#define CARTESIAN_ROBOT_H_

#include "descartes_core/robot_model.h"

namespace descartes_trajectory_test
{
/**@brief Cartesian Robot used for test purposes.  This is a simple robot with simple kinematics.  Each
 * joint corresponds to a cartesian direction (i.e. x, y, R, P, Y) (don't ask me how this is built, it
 * just works).
*/
class CartesianRobot : public descartes_core::RobotModel
{
public:
  CartesianRobot();

  CartesianRobot(double pos_range, double orient_range,
                 const std::vector<double> &joint_velocities = std::vector<double>(6, 1.0));

  virtual bool getIK(const Eigen::Affine3d &pose, const std::vector<double> &seed_state,
                     std::vector<double> &joint_pose) const;

  virtual bool getAllIK(const Eigen::Affine3d &pose, std::vector<std::vector<double> > &joint_poses) const;

  virtual bool getFK(const std::vector<double> &joint_pose, Eigen::Affine3d &pose) const;

  virtual bool isValid(const std::vector<double> &joint_pose) const;

  virtual bool isValid(const Eigen::Affine3d &pose) const;

  virtual int getDOF() const;

  virtual bool initialize(const std::string &robot_description, const std::string &group_name,
                          const std::string &world_frame, const std::string &tcp_frame);

  virtual bool isValidMove(const std::vector<double> &from_joint_pose, const std::vector<double> &to_joint_pose,
                           double dt) const;

  bool setJointVelocities(const std::vector<double> &new_joint_vels)
  {
    if (new_joint_vels.size() != joint_velocities_.size())
      return false;
    joint_velocities_ = new_joint_vels;
    return true;
  }

  double pos_range_;
  double orient_range_;
  std::vector<double> joint_velocities_;
};
}

#endif  // CARTESIAN_ROBOT_H
