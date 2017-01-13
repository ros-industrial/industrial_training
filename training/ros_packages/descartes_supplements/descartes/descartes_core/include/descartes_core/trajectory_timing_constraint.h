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
/*
 * trajectory_pt_transition.h
 *
 *  Created on: March 7, 2015
 *      Author: Jonathan Meyer
 */

#ifndef TRAJECTORY_TIMING_CONSTRAINT_H
#define TRAJECTORY_TIMING_CONSTRAINT_H

#include <ros/console.h>

namespace descartes_core
{
/**
 * @brief A window of time for this point to be achieved relative to a previous point or
 *        the starting position.
 *
 * This struct defines a 'lower' and 'upper' bound for the desired time. If the upper bound
 * is zero or negative, it is considered unspecified and the behavior of your planner
 * or filter is considered undefined.
 *
 * All time-value units are in seconds.
 */
struct TimingConstraint
{
  /**
   * @brief The default constructor creates an unspecified point
   */
  TimingConstraint() : lower(0.0), upper(0.0)
  {
  }

  /**
   * @brief Constructs a timing constraint with a nominal time value (window width of zero)
   * @param nominal The desired time in seconds to achieve this point from the previous
   */
  explicit TimingConstraint(double nominal) : lower(nominal), upper(nominal)
  {
    if (nominal < 0.0)
    {
      ROS_WARN_STREAM("Nominal time value must be greater than or equal to 0.0");
      lower = 0.0;
      upper = 0.0;
    }
  }

  /**
   * @brief Constructs a timing constraint using the provided time window
   * @param lower The lower bound of the acceptable time window in seconds.
   * @param upper The upper bound of the acceptable time window in seconds.
   */
  TimingConstraint(double lower, double upper) : lower(lower), upper(upper)
  {
    if (lower < 0.0)
    {
      ROS_WARN_STREAM("Lower time value must be greater than or equal to 0.0");
      lower = 0.0;
    }

    if (upper < 0.0)
    {
      ROS_WARN_STREAM("Upper time value must be greater than or equal to 0.0");
      upper = 0.0;
    }
  }

  /**
   * @brief Checks if the given timing constraint has been specified
   * @return true if the point is specified
   */
  bool isSpecified() const
  {
    return upper > 0.0;
  }

  double lower, upper;
};

}  // end namespace descartes core

#endif /* TRAJECTORY_TIMING_CONSTRAINT_H */
