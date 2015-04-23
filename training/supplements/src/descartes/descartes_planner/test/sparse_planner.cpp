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

#include <ros/node_handle.h>
#include <descartes_planner/sparse_planner.h>
#include <descartes_trajectory/cart_trajectory_pt.h>
#include <descartes_core/utils.h>
#include <descartes_trajectory_test/cartesian_robot.h>
#include <gtest/gtest.h>
#include <descartes_core/pretty_print.hpp>
#include <tuple>

using namespace descartes_core;
using namespace descartes_trajectory;
typedef std::vector<descartes_core::TrajectoryPtPtr> Trajectory;
const int NUM_DENSE_POINTS = 1000;
Trajectory createTestTrajectory();
Trajectory TEST_TRAJECTORY = createTestTrajectory();
descartes_planner::SparsePlanner Planner;

class ThreeDOFRobot: public descartes_trajectory_test::CartesianRobot
{
public:
  ThreeDOFRobot():
    descartes_trajectory_test::CartesianRobot(0,0,3)
  {

  }

  virtual ~ThreeDOFRobot()
  {

  }

};

class TestPoint: public descartes_trajectory::CartTrajectoryPt
{
public:
  TestPoint(const std::vector<double>& joints)
  {
    vals_.resize(joints.size());
    vals_.assign(joints.begin(),joints.end());
  }

  virtual ~TestPoint()
  {

  }

  virtual bool getClosestJointPose(const std::vector<double> &seed_state,
                                     const RobotModel &model,
                                     std::vector<double> &joint_pose) const
  {
    joint_pose.clear();
    joint_pose.assign(vals_.begin(),vals_.end());
    return true;
  }

  virtual void getJointPoses(const RobotModel &model,
                                       std::vector<std::vector<double> > &joint_poses) const
  {
    joint_poses.clear();
    joint_poses.push_back(vals_);

  }

protected:

  std::vector<double> vals_;
};

Trajectory createTestTrajectory()
{
  ROS_INFO_STREAM("Creating test trajectory with "<<NUM_DENSE_POINTS<<" points");
  Trajectory traj;
  std::vector<std::tuple<double, double>>joint_bounds = {std::make_tuple(0,M_PI),
                                                         std::make_tuple(-M_PI_2,M_PI_2),
                                                         std::make_tuple(M_PI/8,M_PI/3)};
  std::vector<double> deltas;
  for(auto& e:joint_bounds)
  {
    double d = (std::get<1>(e)- std::get<0>(e))/NUM_DENSE_POINTS;
    deltas.push_back(d);
  }

  // creating trajectory points
  std::vector<double> joint_vals(deltas.size(),0);
  traj.reserve(NUM_DENSE_POINTS);
  for(int i = 0 ; i < NUM_DENSE_POINTS; i++)
  {
    for(int j = 0; j < deltas.size(); j++)
    {
      joint_vals[j] = std::get<0>(joint_bounds[j]) + deltas[j]*i;
    }

    TrajectoryPtPtr tp(new TestPoint(joint_vals));
    traj.push_back(tp);
  }
  return traj;
}

TEST(SparsePlanner, initialize)
{
  ros::Time::init();
  RobotModelConstPtr robot(new ThreeDOFRobot());
  EXPECT_TRUE(Planner.initialize(robot));
}

TEST(SparsePlanner, configure)
{
  descartes_core::PlannerConfig config;
  Planner.getConfig(config);
  EXPECT_TRUE(Planner.setConfig(config));
}


TEST(SparsePlanner, planPath)
{
  ROS_INFO_STREAM("Testing planPath() with "<<NUM_DENSE_POINTS<<" points");
  EXPECT_TRUE(Planner.planPath(TEST_TRAJECTORY));
}

TEST(SparsePlanner, getPath)
{
  std::vector<descartes_core::TrajectoryPtPtr> path;
  EXPECT_TRUE(Planner.getPath(path));
  EXPECT_TRUE(path.size() == NUM_DENSE_POINTS);
}


