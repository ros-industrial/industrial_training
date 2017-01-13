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
#include "descartes_trajectory_test/robot_model_test.hpp"

using namespace descartes_core;

using testing::Types;

namespace descartes_trajectory_test
{
template <>
RobotModelPtr CreateRobotModel<CartesianRobot>()
{
  return RobotModelPtr(new CartesianRobot());
}

template <class T>
class CartesianRobotModelTest : public descartes_trajectory_test::RobotModelTest<T>
{
};

INSTANTIATE_TYPED_TEST_CASE_P(CartesianRobotModelTest, RobotModelTest, CartesianRobot);

}  // descartes_trajectory_test
