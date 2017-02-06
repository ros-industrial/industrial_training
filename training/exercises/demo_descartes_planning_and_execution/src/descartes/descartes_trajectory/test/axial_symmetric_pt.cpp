/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2015, Southwest Research Institute
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

#include "descartes_trajectory/axial_symmetric_pt.h"
#include "descartes_core/utils.h"
#include "descartes_trajectory_test/cartesian_robot.h"
#include <gtest/gtest.h>

using namespace descartes_trajectory;
using namespace descartes_trajectory_test;

// Does it have a default constructor
TEST(AxialSymPt, construction)
{
  AxialSymmetricPt def();
}

TEST(AxialSymPt, discretization_count)
{
  const double SEARCH_DISC = M_PI / 2.0;
  CartesianRobot robot (10.0, 2*M_PI);

  Eigen::Affine3d pose (Eigen::Affine3d::Identity());

  AxialSymmetricPt z_point (pose, SEARCH_DISC, AxialSymmetricPt::Z_AXIS);
  AxialSymmetricPt x_point (pose, SEARCH_DISC, AxialSymmetricPt::X_AXIS);
  AxialSymmetricPt y_point (pose, SEARCH_DISC, AxialSymmetricPt::Y_AXIS);

  EigenSTL::vector_Affine3d solutions;
  std::vector<std::vector<double> > joint_solutions;

  const unsigned EXPECTED_POSES = (2.0 * M_PI / SEARCH_DISC) + 1;

  z_point.getCartesianPoses(robot, solutions);
  EXPECT_EQ(EXPECTED_POSES, solutions.size());
  z_point.getJointPoses(robot, joint_solutions);
  EXPECT_EQ(EXPECTED_POSES, joint_solutions.size());

  x_point.getCartesianPoses(robot, solutions);
  EXPECT_EQ(EXPECTED_POSES, solutions.size());
  x_point.getJointPoses(robot, joint_solutions);
  EXPECT_EQ(EXPECTED_POSES, joint_solutions.size());

  y_point.getCartesianPoses(robot, solutions);
  EXPECT_EQ(EXPECTED_POSES, solutions.size());
  y_point.getJointPoses(robot, joint_solutions);
  EXPECT_EQ(EXPECTED_POSES, joint_solutions.size());
}

bool approxEqual(double a, double b, double tol)
{
  return std::fmod(std::fabs(a - b), M_PI) <= tol;
}

TEST(AxialSymPt, discretization_values)
{
  const double SEARCH_DISC = M_PI / 2.0;
  CartesianRobot robot (10.0, 2*M_PI);

  Eigen::Affine3d pose (Eigen::Affine3d::Identity());

  AxialSymmetricPt z_point (pose, SEARCH_DISC, AxialSymmetricPt::Z_AXIS);
  AxialSymmetricPt x_point (pose, SEARCH_DISC, AxialSymmetricPt::X_AXIS);
  AxialSymmetricPt y_point (pose, SEARCH_DISC, AxialSymmetricPt::Y_AXIS);

  const double ANGLE_TOL = 0.001;
  // 
  EigenSTL::vector_Affine3d solutions;
  z_point.getCartesianPoses(robot, solutions);
  for (std::size_t i = 0; i < solutions.size(); ++i)
  {
    Eigen::Vector3d rpy = solutions[i].rotation().eulerAngles(0,1,2);
    EXPECT_TRUE(approxEqual(0.0, rpy(0), ANGLE_TOL));
    EXPECT_TRUE(approxEqual(0.0, rpy(1), ANGLE_TOL));
    EXPECT_TRUE(approxEqual(-M_PI + SEARCH_DISC * i, rpy(2), ANGLE_TOL));
  }

  x_point.getCartesianPoses(robot, solutions);
  for (std::size_t i = 0; i < solutions.size(); ++i)
  {
    Eigen::Vector3d rpy = solutions[i].rotation().eulerAngles(0,1,2);
    EXPECT_TRUE(approxEqual(-M_PI + SEARCH_DISC * i, rpy(0), ANGLE_TOL));
    EXPECT_TRUE(approxEqual(0.0, rpy(1), ANGLE_TOL));
    EXPECT_TRUE(approxEqual(0.0, rpy(2), ANGLE_TOL));
  }

  y_point.getCartesianPoses(robot, solutions);
  for (std::size_t i = 0; i < solutions.size(); ++i)
  {
    Eigen::Vector3d rpy = solutions[i].rotation().eulerAngles(0,1,2);
    EXPECT_TRUE(approxEqual(0.0, rpy(0), ANGLE_TOL));
    EXPECT_TRUE(approxEqual(-M_PI + SEARCH_DISC * i, rpy(1), ANGLE_TOL));
    EXPECT_TRUE(approxEqual(0.0, rpy(2), ANGLE_TOL));
  }
}


