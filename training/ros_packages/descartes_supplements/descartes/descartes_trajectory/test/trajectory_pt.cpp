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

#include "descartes_core/trajectory_pt.h"
#include "descartes_trajectory/cart_trajectory_pt.h"
#include "descartes_trajectory/joint_trajectory_pt.h"
#include "descartes_trajectory/axial_symmetric_pt.h"
#include "ros/console.h"
#include <gtest/gtest.h>

using namespace descartes_core;
using namespace descartes_trajectory;

// Helper function for testing timing data
bool equal(const descartes_core::TimingConstraint& a, const descartes_core::TimingConstraint& b)
{
  return std::abs(a.upper - b.upper) < 0.001;
}

// Factory methods for trajectory point construction
template <class T>
TrajectoryPt* CreateTrajectoryPt();

template <>
TrajectoryPt* CreateTrajectoryPt<CartTrajectoryPt>()
{
  return new CartTrajectoryPt();
}

template <>
TrajectoryPt* CreateTrajectoryPt<JointTrajectoryPt>()
{
  return new JointTrajectoryPt();
}

template <>
TrajectoryPt* CreateTrajectoryPt<AxialSymmetricPt>()
{
  return new AxialSymmetricPt();
}

template <class T>
class TrajectoryPtTest : public testing::Test
{
protected:
  TrajectoryPtTest()
    : lhs_(CreateTrajectoryPt<T>())
    , rhs_(CreateTrajectoryPt<T>())
    , lhs_copy_(CreateTrajectoryPt<T>())
    , lhs_clone_(CreateTrajectoryPt<T>())
  {
    lhs_->setTiming(descartes_core::TimingConstraint(10.0));

    lhs_copy_ = lhs_->copy();
    lhs_clone_ = lhs_->clone();
    lhs_same_ = lhs_;
  }

  TrajectoryPtPtr lhs_;
  TrajectoryPtPtr rhs_;
  TrajectoryPtPtr lhs_copy_;
  TrajectoryPtPtr lhs_clone_;
  TrajectoryPtPtr lhs_same_;
};

using testing::Types;

// Add types of trajectory points here:
typedef Types<CartTrajectoryPt, JointTrajectoryPt, AxialSymmetricPt> Implementations;

TYPED_TEST_CASE(TrajectoryPtTest, Implementations);

TYPED_TEST(TrajectoryPtTest, construction)
{
  EXPECT_FALSE(this->lhs_->getID().is_nil());
  EXPECT_FALSE(this->lhs_copy_->getID().is_nil());
  EXPECT_FALSE(this->lhs_clone_->getID().is_nil());
  EXPECT_FALSE(this->rhs_->getID().is_nil());

  // Depending on construction method (declaration, copy, clone, same pointer), the
  // objects and specifically IDs equality should be defined as follow

  // TODO: Implement equality checks

  // Declared objects should always be different
  // EXPECT_NE(*(this->lhs_), *(this->rhs_));
  EXPECT_NE(this->lhs_->getID(), this->rhs_->getID());

  // Copied objects should always be the same
  // EXPECT_EQ(*(this->lhs_), *(this->lhs_copy_));
  EXPECT_EQ(this->lhs_->getID(), this->lhs_copy_->getID());

  // Cloned objects should have the same data (we can't test, but different ids)
  // EXPECT_NE(*(this->lhs_), *(this->lhs_clone_));
  EXPECT_NE(this->lhs_->getID(), this->lhs_clone_->getID());

  // Pointers to the same objects should be identical (like a copy, but no ambiguity)
  // EXPECT_EQ(*(this->lhs_), *(this->lhs_same_));
  EXPECT_EQ(this->lhs_->getID(), this->lhs_same_->getID());

  // Copied and cloned points should have the same timing information
  EXPECT_TRUE(equal(this->lhs_->getTiming(), this->lhs_copy_->getTiming()));
  EXPECT_TRUE(equal(this->lhs_->getTiming(), this->lhs_clone_->getTiming()));
  EXPECT_FALSE(equal(this->lhs_->getTiming(), this->rhs_->getTiming()));
}
