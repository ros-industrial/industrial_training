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

#include "descartes_trajectory_test/cartesian_robot.h"
#include "descartes_core/pretty_print.hpp"
#include "eigen_conversions/eigen_kdl.h"
#include "ros/console.h"

namespace descartes_trajectory_test
{

  CartesianRobot::CartesianRobot() : pos_range_(2.0), orient_range_(M_PI_2), dof_(6)
  {
    ROS_DEBUG_STREAM("Creating cartesian robot with range, position: " << pos_range_
                     << ", orientation: " << orient_range_);
  }

  bool CartesianRobot::initialize(const std::string& robot_description, const std::string& group_name,
                                  const std::string& world_frame,const std::string& tcp_frame)
  {
    return true;
  }

  CartesianRobot::CartesianRobot(double pos_range, double orient_range, int dof) :
    pos_range_(pos_range), orient_range_(orient_range), dof_(dof)
  {
    ROS_DEBUG_STREAM("Creating cartesian robot with range, position: " << pos_range_
                     << ", orientation: " << orient_range_);
  }

  bool CartesianRobot::getIK(const Eigen::Affine3d &pose, const std::vector<double> &seed_state,
                     std::vector<double> &joint_pose) const
  {
    bool rtn = false;
    KDL::Frame kdl_frame;
    tf::transformEigenToKDL(pose, kdl_frame);

    joint_pose.resize(6, 0.0);
    joint_pose[0] = kdl_frame.p.x();
    joint_pose[1] = kdl_frame.p.y();
    joint_pose[2] = kdl_frame.p.z();
    kdl_frame.M.GetRPY(joint_pose[3], joint_pose[4], joint_pose[5]);

    if(isValid(joint_pose))
    {
      rtn = true;
    }
    else
    {
      rtn = false;
    }
    return rtn;
  }

  bool CartesianRobot::getAllIK(const Eigen::Affine3d &pose,
                                std::vector<std::vector<double> > &joint_poses) const
  {
    std::vector<double>empty;
    joint_poses.resize(1);
    return getIK(pose, empty, joint_poses[0]);
  }

  bool CartesianRobot::getFK(const std::vector<double> &joint_pose, Eigen::Affine3d &pose) const
  {
    bool rtn = false;

    if(isValid(joint_pose))
    {
      pose = Eigen::Translation3d(joint_pose[0], joint_pose[1], joint_pose[2]) *
          Eigen::AngleAxisd(joint_pose[5], Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(joint_pose[4], Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(joint_pose[3], Eigen::Vector3d::UnitX());
      rtn = true;
    }
    else
    {
      ROS_WARN_STREAM("Invalid joint pose passed to get FK, joint pose" << joint_pose);
      rtn = false;
    }

    return rtn;
  }

  int CartesianRobot::getDOF() const
  {
    return dof_;
  }

  bool CartesianRobot::isValid(const std::vector<double> &joint_pose) const
  {
    bool rtn = false;

    double pos_limit = pos_range_/2.0;
    double orient_limit = orient_range_/2.0;

    if(dof_ == joint_pose.size())
    {

      rtn = ( fabs(joint_pose[0]) <= pos_limit &&
              fabs(joint_pose[1]) <= pos_limit &&
              fabs(joint_pose[2]) <= pos_limit &&
              fabs(joint_pose[3]) <= orient_limit &&
              fabs(joint_pose[4]) <= orient_limit &&
              fabs(joint_pose[5]) <= orient_limit );
    }
    else
    {
      ROS_DEBUG_STREAM("Joint pose size: " << joint_pose.size() << "exceeds "<<dof_);
    }

    return rtn;
  }

  bool CartesianRobot::isValid(const Eigen::Affine3d &pose) const
  {
    bool rtn = false;
    double R, P, Y;
    KDL::Frame kdl_frame;
    tf::transformEigenToKDL(pose, kdl_frame);
    kdl_frame.M.GetRPY(R, P, Y);

    double pos_limit = pos_range_/2.0;
    double orient_limit = orient_range_/2.0;

    rtn = ( fabs(kdl_frame.p.x()) <= pos_limit &&
            fabs(kdl_frame.p.y()) <= pos_limit &&
            fabs(kdl_frame.p.z()) <= pos_limit &&
            fabs(R) <= orient_limit &&
            fabs(P) <= orient_limit &&
            fabs(Y) <= orient_limit );

    return rtn;
  }


} //descartes_trajectory_test

