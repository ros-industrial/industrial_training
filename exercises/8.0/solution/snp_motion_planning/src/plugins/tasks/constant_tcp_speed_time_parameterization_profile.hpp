#pragma once

#include <memory>
#include <cmath>

static const std::string CONSTANT_TCP_SPEED_TIME_PARAM_TASK_NAME = "ConstantTCPSpeedTimeParameterizationTask";

namespace snp_motion_planning
{
struct ConstantTCPSpeedTimeParameterizationProfile
{
  using Ptr = std::shared_ptr<ConstantTCPSpeedTimeParameterizationProfile>;
  using ConstPtr = std::shared_ptr<const ConstantTCPSpeedTimeParameterizationProfile>;

  ConstantTCPSpeedTimeParameterizationProfile() = default;
  ConstantTCPSpeedTimeParameterizationProfile(double max_translational_velocity_, double max_rotational_velocity_,
                                              double max_translational_acceleration_,
                                              double max_rotational_acceleration_,
                                              double max_velocity_scaling_factor_ = 1.0,
                                              double max_acceleration_scaling_factor_ = 1.0)
    : max_translational_velocity(max_translational_velocity_)
    , max_rotational_velocity(max_rotational_velocity_)
    , max_translational_acceleration(max_translational_acceleration_)
    , max_rotational_acceleration(max_rotational_acceleration_)
    , max_velocity_scaling_factor(max_velocity_scaling_factor_)
    , max_acceleration_scaling_factor(max_acceleration_scaling_factor_)
  {
  }

  double max_translational_velocity;
  double max_rotational_velocity;
  double max_translational_acceleration;
  double max_rotational_acceleration;
  double max_velocity_scaling_factor;
  double max_acceleration_scaling_factor;
};

}  // namespace snp_motion_planning
