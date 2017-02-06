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

#include "descartes_trajectory/cart_trajectory_pt.h"
#include "descartes_core/utils.h"
#include "descartes_trajectory_test/cartesian_robot.h"
#include <gtest/gtest.h>


using namespace descartes_trajectory;
using namespace descartes_core;
using namespace descartes_trajectory_test;



TEST(CartTrajPt, construction)
{
  CartTrajectoryPt def();
}

TEST(CartTrajPt, getPoses)
{
  const double POS_TOL = 2.0;
  const double POS_INC = 0.2;

  const double ORIENT_TOL = 2*M_PI;
  const double ORIENT_INC = M_PI/4;

  const double EPSILON = 0.001;

  const int NUM_SAMPLED_POS = pow((POS_TOL/POS_INC) + 1, 3.0);
  const int NUM_SAMPLED_ORIENT = pow((ORIENT_TOL/ORIENT_INC) + 1, 3.0);
  const int NUM_SAMPLED_BOTH = NUM_SAMPLED_POS * NUM_SAMPLED_ORIENT;

  ROS_INFO_STREAM("Expected samples, position: " << NUM_SAMPLED_POS
                  << ", orientation: " << NUM_SAMPLED_ORIENT
                  << ", both: " << NUM_SAMPLED_BOTH);

  ROS_INFO_STREAM("Initializing fuzzy position point");
  CartTrajectoryPt fuzzy_pos(TolerancedFrame(
                               utils::toFrame(0, 0, 0, 0, 0, 0),
                               ToleranceBase::createSymmetric<PositionTolerance>(0.0, 0.0, 0.0, POS_TOL + EPSILON),
                               ToleranceBase::createSymmetric<OrientationTolerance>(0.0, 0.0, 0.0, 0.0)),
                             POS_INC, ORIENT_INC);

  ROS_INFO_STREAM("Initializing fuzzy orientation point");
  CartTrajectoryPt fuzzy_orient(TolerancedFrame(
                                  utils::toFrame(0, 0, 0, 0, 0, 0),
                                  ToleranceBase::createSymmetric<PositionTolerance>(0.0, 0.0, 0.0, 0.0),
                                  ToleranceBase::createSymmetric<OrientationTolerance>(0.0, 0.0, 0.0, ORIENT_TOL + EPSILON)),
                                POS_INC, ORIENT_INC);

  ROS_INFO_STREAM("Initializing fuzzy position/orientation point");
  CartTrajectoryPt fuzzy_both(TolerancedFrame(
                                utils::toFrame(0, 0, 0, 0, 0, 0),
                                ToleranceBase::createSymmetric<PositionTolerance>(0.0, 0.0, 0.0, POS_TOL + EPSILON),
                                ToleranceBase::createSymmetric<OrientationTolerance>(0.0, 0.0, 0.0, ORIENT_TOL + EPSILON)),
                              POS_INC, ORIENT_INC);


  EigenSTL::vector_Affine3d solutions;
  std::vector<std::vector<double> >joint_solutions;

  ROS_INFO_STREAM("Testing fuzzy pos point");
  CartesianRobot robot(POS_TOL+2*EPSILON, ORIENT_TOL+2*EPSILON);
  fuzzy_pos.getCartesianPoses(robot, solutions);
  EXPECT_EQ(solutions.size(), NUM_SAMPLED_POS);
  fuzzy_pos.getJointPoses(robot,joint_solutions);
  EXPECT_EQ(joint_solutions.size(), NUM_SAMPLED_POS);

  ROS_INFO_STREAM("Testing fuzzy orient point");
  fuzzy_orient.getCartesianPoses(robot, solutions);
  EXPECT_EQ(solutions.size(), NUM_SAMPLED_ORIENT);
  fuzzy_orient.getJointPoses(robot,joint_solutions);
  EXPECT_EQ(joint_solutions.size(), NUM_SAMPLED_ORIENT);

  ROS_INFO_STREAM("Testing fuzzy both point");
  fuzzy_both.getCartesianPoses(robot, solutions);
  EXPECT_EQ(solutions.size(), NUM_SAMPLED_BOTH);
  fuzzy_both.getJointPoses(robot,joint_solutions);
  EXPECT_EQ(joint_solutions.size(), NUM_SAMPLED_BOTH);

  // TODO: Add tests for cartesian points outside of the robot workspace.  These
  // tests below seem to work, but predicting the number of samples isn't working.
  //  const double WS_FRACTION = 0.5;  //Reduces robot workspace
  //  const int LIMIT_SAMPLED_POS = pow(((WS_FRACTION * POS_TOL)/POS_INC) + 1, 3.0);
  //  const int LIMIT_SAMPLED_ORIENT = pow(((WS_FRACTION * ORIENT_TOL)/ORIENT_INC) + 1, 3.0);
  //  const int LIMIT_SAMPLED_BOTH = LIMIT_SAMPLED_POS * LIMIT_SAMPLED_ORIENT;

  //  CartesianRobot limited_robot(POS_TOL * WS_FRACTION + 2*EPSILON, ORIENT_TOL * WS_FRACTION + 2*EPSILON);
  //  fuzzy_pos.getCartesianPoses(limited_robot, solutions);
  //  EXPECT_EQ(solutions.size(), LIMIT_SAMPLED_POS);

  //  fuzzy_orient.getCartesianPoses(limited_robot, solutions);
  //  EXPECT_EQ(solutions.size(), LIMIT_SAMPLED_ORIENT);

  //  fuzzy_both.getCartesianPoses(limited_robot, solutions);
  //  EXPECT_EQ(solutions.size(), LIMIT_SAMPLED_BOTH);

}


TEST(CartTrajPt, zeroTolerance)
{

  ROS_INFO_STREAM("Initializing zero tolerance cartesian point");
  CartTrajectoryPt zero_tol_pos(TolerancedFrame(
                               utils::toFrame(0, 0, 0, 0, 0, 0),
                               ToleranceBase::zeroTolerance<PositionTolerance>(0.0, 0.0, 0.0),
                               ToleranceBase::zeroTolerance<OrientationTolerance>(0.0, 0.0, 0.0)),
                             0, 0);

  EigenSTL::vector_Affine3d solutions;
  std::vector<std::vector<double> >joint_solutions;

  CartesianRobot robot;
  zero_tol_pos.getCartesianPoses(robot, solutions);
  EXPECT_EQ(solutions.size(), 1);
  zero_tol_pos.getJointPoses(robot,joint_solutions);
  EXPECT_EQ(joint_solutions.size(), 1);
}

TEST(CartTrajPt, closestJointPose)
{
  const double POS_TOL = 0.5f;
  const double POS_INC = 0.2;
  const double ORIENT_TOL = 1.0;
  const double ORIENT_INC = 0.2;
  CartesianRobot robot(10,4);

  // declaring pose values
  const double x = 4.0f;
  const double y = 5.0f;
  const double z = 2.0f;
  const double rx = 0.0f;
  const double ry = 0.0f;
  const double rz = M_PI/4;
  std::vector<double> joint_pose                                                                                                                                                                                                                                                                                                                                                                                                                                                                = {x,y,z,rx,ry,rz};
  Eigen::Affine3d frame_pose = descartes_core::utils::toFrame(x,y,z,rx,ry,rz);

  ROS_INFO_STREAM("Initializing tolerance cartesian point");
  CartTrajectoryPt cart_point(TolerancedFrame(
                                utils::toFrame(x,y,z,rx,ry,rz),
                                ToleranceBase::createSymmetric<PositionTolerance>(x,y,z, POS_TOL),
                                ToleranceBase::createSymmetric<OrientationTolerance>(rx,ry,rz, ORIENT_TOL)),
                              POS_INC, ORIENT_INC);

  ROS_INFO_STREAM("Testing getClosestJointPose(...)");
  std::vector<double> closest_joint_pose;
  EXPECT_TRUE(cart_point.getClosestJointPose(joint_pose,robot,closest_joint_pose));

  ROS_INFO_STREAM("Testing equality between seed joint pose and closest joint pose");
  EXPECT_EQ(joint_pose,closest_joint_pose);
}
