#include <gtest/gtest.h>

#include <ros/console.h>
#include <descartes_core/path_planner_base.h>
#include <descartes_trajectory_test/cartesian_robot.h>

#include "utils/trajectory_maker.h"

// Google Test Framework for Path Planners

template <class T>
class PathPlannerTest : public testing::Test
{
protected:
  descartes_core::PathPlannerBasePtr makePlanner()
  {
    descartes_core::PathPlannerBasePtr planner(new T());
    EXPECT_TRUE(planner->initialize(robot_)) << "Failed to initalize robot model";
    return planner;
  }

  PathPlannerTest()
    : velocity_limits_(6, 1.0), robot_(new descartes_trajectory_test::CartesianRobot(5.0, 0.001, velocity_limits_))
  {
    ros::Time::init();
  }

  std::vector<double> velocity_limits_;
  descartes_core::RobotModelConstPtr robot_;
};

TYPED_TEST_CASE_P(PathPlannerTest);

TYPED_TEST_P(PathPlannerTest, construction)
{
  using namespace descartes_core;

  PathPlannerBasePtr planner = this->makePlanner();
}

TYPED_TEST_P(PathPlannerTest, basicConfigure)
{
  descartes_core::PathPlannerBasePtr planner = this->makePlanner();
  descartes_core::PlannerConfig config;
  planner->getConfig(config);
  EXPECT_TRUE(planner->setConfig(config));
}

TYPED_TEST_P(PathPlannerTest, preservesTiming)
{
  using namespace descartes_tests;
  using namespace descartes_core;

  PathPlannerBasePtr planner = this->makePlanner();

  std::vector<TrajectoryPtPtr> input, output;
  // Make input trajectory
  input = makeConstantVelocityTrajectory(Eigen::Vector3d(-1.0, 0, 0),  // start position
                                         Eigen::Vector3d(1.0, 0, 0),   // end position
                                         0.9,                          // tool velocity
                                         20);                          // samples
  // Double the dt of every pt to provide some variety
  double dt = input.front().get()->getTiming().upper;
  for (auto& pt : input)
  {
    pt.get()->setTiming(TimingConstraint(dt));
    dt *= 1.5;
  }
  // // Solve
  ASSERT_TRUE(planner->planPath(input));
  // Get the result
  ASSERT_TRUE(planner->getPath(output));
  // Compare timing
  ASSERT_TRUE(input.size() == output.size());
  for (size_t i = 0; i < input.size(); ++i)
  {
    double t1 = input[i].get()->getTiming().upper;
    double t2 = output[i].get()->getTiming().upper;
    EXPECT_DOUBLE_EQ(t1, t2) << "Input/output timing should correspond for same index: " << t1 << " " << t2;
  }
}

TYPED_TEST_P(PathPlannerTest, simpleVelocityCheck)
{
  using namespace descartes_core;

  PathPlannerBasePtr planner = this->makePlanner();

  std::vector<descartes_core::TrajectoryPtPtr> input;
  input = descartes_tests::makeConstantVelocityTrajectory(Eigen::Vector3d(-1.0, 0, 0),  // start position
                                                          Eigen::Vector3d(1.0, 0, 0),   // end position
                                                          0.9,  // tool velocity (< 1.0 m/s limit)
                                                          10);  // samples
  ASSERT_TRUE(!input.empty());
  // The nominal trajectory (0.9 m/s) is less than max tool speed of 1.0 m/s
  EXPECT_TRUE(planner->planPath(input));
  // Unconstraining a point should still succeed
  input.back().get()->setTiming(descartes_core::TimingConstraint());
  EXPECT_TRUE(planner->planPath(input));
  // Making a dt for a segment very small should induce failure
  input.back().get()->setTiming(descartes_core::TimingConstraint(0.001));
  EXPECT_FALSE(planner->planPath(input)) << "Trajectory pt has very small dt; planner should fail for velocity out of "
                                            "bounds";
}

TYPED_TEST_P(PathPlannerTest, zigzagTrajectory)
{
  using namespace descartes_core;

  PathPlannerBasePtr planner = this->makePlanner();

  std::vector<descartes_core::TrajectoryPtPtr> input;
  input = descartes_tests::makeZigZagTrajectory(-1.0,  // start position
                                                1.0,   // end position
                                                0.5,
                                                0.1,  // tool velocity (< 1.0 m/s limit)
                                                10);  // samples
  ASSERT_TRUE(!input.empty());
  // The nominal trajectory (0.9 m/s) is less than max tool speed of 1.0 m/s
  EXPECT_TRUE(planner->planPath(input));
}

REGISTER_TYPED_TEST_CASE_P(PathPlannerTest, construction, basicConfigure, preservesTiming, simpleVelocityCheck,
                           zigzagTrajectory);
