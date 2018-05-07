#include "descartes_trajectory/axial_symmetric_pt.h"

using descartes_trajectory::TolerancedFrame;
using descartes_trajectory::AxialSymmetricPt;
using namespace descartes_core::utils;

static TolerancedFrame makeUnconstrainedRotation(double x, double y, double z, double rx, double ry, double rz,
                                                 AxialSymmetricPt::FreeAxis axis)
{
  using namespace descartes_trajectory;

  Eigen::Affine3d pose = toFrame(x, y, z, rx, ry, rz, EulerConventions::XYZ);
  PositionTolerance pos_tol = ToleranceBase::zeroTolerance<PositionTolerance>(x, y, z);
  OrientationTolerance orient_tol = ToleranceBase::createSymmetric<OrientationTolerance>(
      ((axis == AxialSymmetricPt::X_AXIS) ? 0.0 : rx), ((axis == AxialSymmetricPt::Y_AXIS) ? 0.0 : ry),
      ((axis == AxialSymmetricPt::Z_AXIS) ? 0.0 : rz), ((axis == AxialSymmetricPt::X_AXIS) ? 2 * M_PI : 0.0),
      ((axis == AxialSymmetricPt::Y_AXIS) ? 2 * M_PI : 0.0), ((axis == AxialSymmetricPt::Z_AXIS) ? 2 * M_PI : 0.0));
  return TolerancedFrame(pose, pos_tol, orient_tol);
}

static TolerancedFrame makeUnconstrainedRotation(const Eigen::Affine3d& pose, AxialSymmetricPt::FreeAxis axis)
{
  using namespace descartes_trajectory;

  Eigen::Vector3d rpy = pose.rotation().eulerAngles(0, 1, 2);
  double rx = rpy(0);
  double ry = rpy(1);
  double rz = rpy(2);
  double x = pose.translation()(0);
  double y = pose.translation()(1);
  double z = pose.translation()(2);

  PositionTolerance pos_tol = ToleranceBase::zeroTolerance<PositionTolerance>(x, y, z);
  OrientationTolerance orient_tol = ToleranceBase::createSymmetric<OrientationTolerance>(
      ((axis == AxialSymmetricPt::X_AXIS) ? 0.0 : rx), ((axis == AxialSymmetricPt::Y_AXIS) ? 0.0 : ry),
      ((axis == AxialSymmetricPt::Z_AXIS) ? 0.0 : rz), ((axis == AxialSymmetricPt::X_AXIS) ? 2 * M_PI : 0.0),
      ((axis == AxialSymmetricPt::Y_AXIS) ? 2 * M_PI : 0.0), ((axis == AxialSymmetricPt::Z_AXIS) ? 2 * M_PI : 0.0));
  return TolerancedFrame(pose, pos_tol, orient_tol);
}

namespace descartes_trajectory
{
AxialSymmetricPt::AxialSymmetricPt(const descartes_core::TimingConstraint& timing) : CartTrajectoryPt(timing)
{
}

AxialSymmetricPt::AxialSymmetricPt(double x, double y, double z, double rx, double ry, double rz,
                                   double orient_increment, FreeAxis axis,
                                   const descartes_core::TimingConstraint& timing)
  : CartTrajectoryPt(makeUnconstrainedRotation(x, y, z, rx, ry, rz, axis),
                     0.0,               // The position discretization
                     orient_increment,  // Orientation discretization (starting at -2Pi and marching to 2Pi)
                     timing)
{
}

AxialSymmetricPt::AxialSymmetricPt(const Eigen::Affine3d& pose, double orient_increment, FreeAxis axis,
                                   const descartes_core::TimingConstraint& timing)
  : CartTrajectoryPt(makeUnconstrainedRotation(pose, axis),
                     0.0,               // The position discretization
                     orient_increment,  // Orientation discretization (starting at -2Pi and marching to 2Pi)
                     timing)
{
}

}  // end of namespace descartes_trajectory
