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

#ifndef ROBOT_MODEL_TEST_HPP_
#define ROBOT_MODEL_TEST_HPP_

#include "descartes_core/pretty_print.hpp"
#include "descartes_core/robot_model.h"
#include "ros/console.h"
#include <gtest/gtest.h>

/**
  @brief: This file contains inerface test for the Descartes RobotModel.  These
  tests can be executed on arbitrary types that inplment the RobotModel interface.
  For more info see:
  <a href="https://code.google.com/p/googletest/wiki/AdvancedGuide#Type-Parameterized_Tests">
    GTest Advanced Guide - Type-Parameterized Tests
  </a>
  */

namespace descartes_trajectory_test
{

template <class T>
descartes_core::RobotModelPtr CreateRobotModel();

template <class T>
class RobotModelTest : public ::testing::Test
{
public:
  RobotModelTest() : model_(CreateRobotModel<T>())
  {
    ROS_INFO("Instantiated RobotModelTest fixture(base) (parameterized)");
  }

  virtual void SetUp()
  {
    ROS_INFO("Setting up RobotModelTest fixture(base) (parameterized)");
    ASSERT_TRUE(static_cast<bool>(this->model_));
  }

  virtual void TearDown()
  {
    ROS_INFO("Tearing down RobotModelTest fixture(base) (parameterized)");
  }

  virtual ~RobotModelTest()
  {
    ROS_INFO("Desctructing RobotModelTest fixture(base) (parameterized)");
  }

  descartes_core::RobotModelPtr model_;
};

using namespace descartes_core;

TYPED_TEST_CASE_P(RobotModelTest);

const double TF_EQ_TOL = 0.001;
const double JOINT_EQ_TOL = 0.001;


TYPED_TEST_P(RobotModelTest, construction) {
  ROS_INFO_STREAM("Robot model test construction");
}


TYPED_TEST_P(RobotModelTest, getIK) {
  ROS_INFO_STREAM("Testing getIK");
  std::vector<double> fk_joint(6, 0.0);
  std::vector<double> ik_joint;
  Eigen::Affine3d ik_pose, fk_pose;
  EXPECT_TRUE(this->model_->getFK(fk_joint, ik_pose));
  EXPECT_TRUE(this->model_->getIK(ik_pose, fk_joint, ik_joint));
  //This doesn't always work, but it should.  The IKFast solution doesn't
  //return the "closets" solution.  Numeric IK does appear to do this.
  EXPECT_TRUE(this->model_->getFK(ik_joint, fk_pose));
  EXPECT_TRUE(ik_pose.matrix().isApprox(fk_pose.matrix(), TF_EQ_TOL));
  ROS_INFO_STREAM("getIK Test completed");
}

TYPED_TEST_P(RobotModelTest, getAllIK) {
  ROS_INFO_STREAM("Testing getAllIK");
  std::vector<double> fk_joint(6, 0.5);
  std::vector<std::vector<double> > joint_poses;
  Eigen::Affine3d ik_pose, fk_pose;

  EXPECT_TRUE(this->model_->getFK(fk_joint, ik_pose));
  EXPECT_TRUE(this->model_->getAllIK(ik_pose, joint_poses));
  ROS_INFO_STREAM("Get all IK returned " << joint_poses.size() << " solutions");
  std::vector<std::vector<double> >::iterator it;
  for (it = joint_poses.begin(); it != joint_poses.end(); ++it)
  {
    ROS_INFO_STREAM("GetIK joint solution: " << *it);
    EXPECT_TRUE(this->model_->getFK(*it, fk_pose));
    EXPECT_TRUE(ik_pose.matrix().isApprox(fk_pose.matrix(), TF_EQ_TOL));
  }
}


REGISTER_TYPED_TEST_CASE_P(RobotModelTest, construction, getIK, getAllIK);

} //descartes_trajectory_test

#endif // ROBOT_MODEL_TEST_HPP_
