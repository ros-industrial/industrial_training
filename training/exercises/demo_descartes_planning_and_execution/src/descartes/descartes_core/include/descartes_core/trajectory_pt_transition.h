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
 *  Created on: Jun 5, 2014
 *      Author: Dan Solomon
 */

#ifndef TRAJECTORY_PT_TRANSITION_H_
#define TRAJECTORY_PT_TRANSITION_H_

#include <boost/shared_ptr.hpp>


namespace descartes_core
{

/**@brief Description of a 1-D velocity constraint
 * Meant to represent either a linear or rotational velocity of the TCP.
 * All velocities are considered positive.
 */
struct VelocityConstraint
{
  VelocityConstraint(): lower(0.), desired(0.), upper(0.) {}
  VelocityConstraint(double _fixed): lower(_fixed), desired(_fixed), upper(_fixed) {}

  double lower, desired, upper;
  /**@brief Check if values are all positive and assigned appropriate value.
   * @return True if all values are positive, lower<=desired && desired <=upper.
   */
  bool check() {return lower>=0. && lower<=desired && desired<=upper;}
};

namespace Interpolations
{
enum Interpolation
{
  DEFAULT = -1, JOINT = 0, CARTESIAN = 1//, CIRCULAR=2
};
}       /* namespace interpolations */
typedef Interpolations::Interpolation Interpolation;

/**@brief TrajectoryPtTransition describes how a state or point is reached from another state/pt.
 * Each point can contain a linear/rotational velocity. (acceleration/jerk are left for future implementation).
 * The velocity is specified AT the point.
 * An interpolation method can be specified that defines how this point is to be reached from the previous point.
 */
class TrajectoryPtTransition
{
public:

  TrajectoryPtTransition(): method_(Interpolations::DEFAULT)
  {
  }

  virtual ~TrajectoryPtTransition()
  {
  }

private:
  VelocityConstraint            lin_vel_;               // Constraint on linear velocity
  VelocityConstraint            rot_vel_;               // Constraint on rotational velocity
  Interpolation                 method_;                // Interpolation method to get to this point

};

typedef boost::shared_ptr<TrajectoryPtTransition> TrajectoryPtTransitionPtr;

} /* namespace descartes_trajectory */


#endif /* TRAJECTORY_PT_TRANSITION_H_ */
