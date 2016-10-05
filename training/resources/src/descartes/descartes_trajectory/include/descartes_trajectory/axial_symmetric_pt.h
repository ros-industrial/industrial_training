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

#ifndef AXIAL_SYMMETRIC_PT_H
#define AXIAL_SYMMETRIC_PT_H

#include <descartes_trajectory/cart_trajectory_pt.h>

namespace descartes_trajectory
{

/**
 * @brief Specialization of cartesian trajectory point.
 *        Represents a cartesian point whose rotation
 *        about a chosen axis is unconstrained.
 */
class AxialSymmetricPt : public descartes_trajectory::CartTrajectoryPt
{
public:

  /**
   * @brief Enum used to select the free axis of rotation for this point
   */
  enum FreeAxis
  {
    X_AXIS,
    Y_AXIS,
    Z_AXIS
  };

  /**
   * @brief This default constructor is exactly equivalent to CartTrajectoryPt's. Initializes all
   *        frames to identity.
   */
  AxialSymmetricPt() {}

  /**
   * @brief Constructs a cartesian trajectory point that places the robot tip at the position
   *        defined by the transform from the origin of the world. The transform is first a
   *        translation by the vector (x,y,z) followed by an XYZ rotation
   *        (moving axis) by (rx,ry,rz) respectively.
   *
   * @param x x component of translation part of transform
   * @param y y component of translation part of transform
   * @param z z component of translation part of transform
   * @param rx rotation about x axis of nominal pose
   * @param ry rotation about y axis of nominal pose
   * @param rz rotation about z axis of nominal pose
   * @param orient_increment (in radians, discretization of space [-2Pi, 2Pi])
   * @param axis The free-axis of the nominal pose of the tool
   */
  AxialSymmetricPt(double x, double y, double z, double rx, double ry, double rz,
                   double orient_increment, FreeAxis axis);

  /**
   * @brief Similar to other constructor except that it takes an affine pose instead of
   *        individual translation and rotation arguments.
   */
  AxialSymmetricPt(const Eigen::Affine3d& pose, double orient_increment, FreeAxis axis);

  virtual descartes_core::TrajectoryPtPtr copy() const
  {
    return descartes_core::TrajectoryPtPtr(new AxialSymmetricPt(*this));
  }

};


} // descartes trajectory

#endif // AXIAL_SYMMETRIC_PT_H
