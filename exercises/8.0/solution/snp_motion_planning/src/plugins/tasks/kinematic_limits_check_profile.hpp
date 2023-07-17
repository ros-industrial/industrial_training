#pragma once

#include <memory>
#include <string>

static const std::string KINEMATIC_LIMITS_CHECK_TASK_NAME = "KinematicLimitsCheckTask";

namespace snp_motion_planning
{
struct KinematicLimitsCheckProfile
{
  using Ptr = std::shared_ptr<KinematicLimitsCheckProfile>;
  using ConstPtr = std::shared_ptr<const KinematicLimitsCheckProfile>;

  KinematicLimitsCheckProfile(bool check_position_ = true, bool check_velocity_ = true, bool check_acceleration_ = true)
    : check_position(check_position_), check_velocity(check_velocity_), check_acceleration(check_acceleration_)
  {
  }

  bool check_position;
  bool check_velocity;
  bool check_acceleration;
};

}  // namespace snp_motion_planning
