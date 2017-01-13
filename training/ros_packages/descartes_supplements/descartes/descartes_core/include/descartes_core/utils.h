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

#ifndef UTILS_H_
#define UTILS_H_

#include <boost/shared_ptr.hpp>
#include <Eigen/Core>

/** \def DESCARTES_CLASS_FORWARD
    Macro that forward declares a class XXX, and also defines two shared ptrs with named XXXPtr and XXXConstPtr  */

#define DESCARTES_CLASS_FORWARD(C)                                                                                     \
  class C;                                                                                                             \
  typedef boost::shared_ptr<C> C##Ptr;                                                                                 \
  typedef boost::shared_ptr<const C> C##ConstPtr;

namespace descartes_core
{
namespace utils
{
/**
  @brief Converts scalar translations and rotations to an Eigen Frame.  This is achieved by chaining a
  translation with individual euler rotations in ZYX order (this is equivalent to fixed rotatins XYZ)
  http://en.wikipedia.org/wiki/Euler_angles#Conversion_between_intrinsic_and_extrinsic_rotations

  @param tx, ty, tz - translations in x, y, z respectively
  @param rx, ry, rz - rotations about x, y, z, respectively
  */
namespace EulerConventions
{
enum EulerConvention
{
  XYZ = 0,
  ZYX,
  ZXZ
};
}

typedef EulerConventions::EulerConvention EulerConvention;

// Use a function declaration so that we can add the 'unused' attribute, which prevents compiler warnings
static Eigen::Affine3d toFrame(double tx, double ty, double tz, double rx, double ry, double rz,
                               int convention = int(EulerConventions::ZYX)) __attribute__((unused));

static Eigen::Affine3d toFrame(double tx, double ty, double tz, double rx, double ry, double rz, int convention)
{
  Eigen::Affine3d rtn;

  switch (convention)
  {
    case EulerConventions::XYZ:
      rtn = Eigen::Translation3d(tx, ty, tz) * Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ());
      break;

    case EulerConventions::ZYX:
      rtn = Eigen::Translation3d(tx, ty, tz) * Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX());
      break;

    case EulerConventions::ZXZ:
      rtn = Eigen::Translation3d(tx, ty, tz) * Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ());
      break;

    default:
      logError("Invalid euler convention entry %i", convention);
      break;
  }

  return rtn;
}

/**
  * Compares two vectors for equality (within +/- tolerance).  abs(lhs - rhs) > tol
  * @param lhs
  * @param rhs
  * @param tol +/- tolerance for floating point equality
  */

// Use a function declaration so that we can add the 'unused' attribute, which prevents compiler warnings
static bool equal(const std::vector<double> &lhs, const std::vector<double> &rhs, const double tol)
    __attribute__((unused));

static bool equal(const std::vector<double> &lhs, const std::vector<double> &rhs, const double tol)
{
  bool rtn = false;
  if (lhs.size() == rhs.size())
  {
    rtn = true;
    for (size_t ii = 0; ii < lhs.size(); ++ii)
    {
      if (std::fabs(lhs[ii] - rhs[ii]) > tol)
      {
        rtn = false;
        break;
      }
    }
  }
  else
  {
    rtn = false;
  }
  return rtn;
}

}  // utils

}  // descartes_core

#endif /* UTILS_H_ */
