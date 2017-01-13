#include "trajectory_maker.h"

// Makes a linear trajectory with a given tool speed
std::vector<descartes_core::TrajectoryPtPtr> descartes_tests::makeConstantVelocityTrajectory(
    const Eigen::Vector3d& start, const Eigen::Vector3d& stop, double tool_vel, size_t n_steps)
{
  Eigen::Vector3d delta = stop - start;
  Eigen::Vector3d step = delta / n_steps;
  double dt = delta.norm() / tool_vel;
  double time_step = dt / n_steps;
  // We know our dt, all points should have it
  descartes_core::TimingConstraint tm(time_step);

  std::vector<descartes_core::TrajectoryPtPtr> result;
  for (size_t i = 0; i <= n_steps; ++i)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(start + i * step);
    descartes_core::TrajectoryPtPtr pt(new descartes_trajectory::CartTrajectoryPt(pose, tm));
    result.push_back(pt);
  }
  return result;
}

std::vector<descartes_core::TrajectoryPtPtr> descartes_tests::makeZigZagTrajectory(double x_start, double x_stop,
                                                                                   double y_amplitude, double tool_vel,
                                                                                   size_t n_steps)
{
  using namespace Eigen;
  Vector3d start(x_start, 0, 0);
  Vector3d stop(x_stop, 0, 0);

  Eigen::Vector3d delta = stop - start;
  Eigen::Vector3d step = delta / n_steps;
  double dt = delta.norm() / tool_vel;
  double time_step = dt / n_steps;
  // We know our dt, all points should have it
  descartes_core::TimingConstraint tm(time_step);

  std::vector<descartes_core::TrajectoryPtPtr> result;
  for (size_t i = 0; i <= n_steps; ++i)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(start + i * step);
    if (i & 1)
    {
      pose *= Translation3d(0, y_amplitude, 0);
    }
    else
    {
      pose *= Translation3d(0, -y_amplitude, 0);
    }
    descartes_core::TrajectoryPtPtr pt(new descartes_trajectory::CartTrajectoryPt(pose, tm));
    result.push_back(pt);
  }
  return result;
}