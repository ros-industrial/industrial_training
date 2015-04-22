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

#include "descartes_moveit/moveit_state_adapter.h"
#include <descartes_trajectory_test/robot_model_test.hpp>
#include "moveit/robot_model_loader/robot_model_loader.h"
#include <gtest/gtest.h>

using namespace descartes_moveit;
using namespace descartes_trajectory;
using namespace descartes_core;

using testing::Types;

namespace descartes_trajectory_test
{
// This variable must be global in order for the test to pass.
// Destructing the robot model results in a boost mutex exception:
// ---
// /usr/include/boost/thread/pthread/pthread_mutex_scoped_lock.hpp:26:
// boost::pthread::pthread_mutex_scoped_lock::pthread_mutex_scoped_lock(pthread_mutex_t*):
// Assertion `!pthread_mutex_lock(m)' failed.
// ---

robot_model_loader::RobotModelLoaderPtr robot_;

template <>
RobotModelPtr CreateRobotModel<descartes_moveit::MoveitStateAdapter>()
{

  ROS_INFO_STREAM("Construction descartes robot model");
  descartes_core::RobotModelPtr descartes_model_;
  descartes_model_ = descartes_core::RobotModelPtr(new descartes_moveit::MoveitStateAdapter());
  EXPECT_TRUE(descartes_model_->initialize("robot_description","manipulator","base_link","tool0"));
  ROS_INFO_STREAM("Descartes robot model constructed");

  descartes_model_->setCheckCollisions(true);
  EXPECT_TRUE(descartes_model_->getCheckCollisions());
  ROS_INFO_STREAM("Descartes robot enabled collision checks");

  return descartes_model_;
}

template<class T>
class MoveitRobotModelTest : public descartes_trajectory_test::RobotModelTest<T>{};

INSTANTIATE_TYPED_TEST_CASE_P(MoveitRobotModelTest, RobotModelTest, MoveitStateAdapter);

} //descartes_moveit_test
