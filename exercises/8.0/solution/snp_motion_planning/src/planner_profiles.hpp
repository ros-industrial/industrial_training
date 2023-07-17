#pragma once

#include <descartes_light/edge_evaluators/compound_edge_evaluator.h>
#include <descartes_light/edge_evaluators/euclidean_distance_edge_evaluator.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>
#include <tesseract_motion_planners/ompl/profile/ompl_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_plan_profile.h>

static const std::string TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask";
static const std::string OMPL_DEFAULT_NAMESPACE = "OMPLMotionPlannerTask";
static const std::string DESCARTES_DEFAULT_NAMESPACE = "DescartesMotionPlannerTask";
static const std::string SIMPLE_DEFAULT_NAMESPACE = "SimpleMotionPlannerTask";
static const std::string MIN_LENGTH_DEFAULT_NAMESPACE = "MinLengthTask";
static const std::string CONTACT_CHECK_DEFAULT_NAMESPACE = "DiscreteContactCheckTask";
static const std::string ISP_DEFAULT_NAMESPACE = "IterativeSplineParameterizationTask";

template <typename FloatType>
typename tesseract_planning::DescartesDefaultPlanProfile<FloatType>::Ptr createDescartesPlanProfile()
{
  auto profile = std::make_shared<tesseract_planning::DescartesDefaultPlanProfile<FloatType>>();

  profile->num_threads = static_cast<int>(std::thread::hardware_concurrency());

  profile->target_pose_sampler =
      std::bind(tesseract_planning::sampleToolZAxis, std::placeholders::_1, 30.0 * M_PI / 180.0);

  return profile;
}

tesseract_planning::OMPLDefaultPlanProfile::Ptr createOMPLProfile()
{
  auto profile = std::make_shared<tesseract_planning::OMPLDefaultPlanProfile>();

  // Give OMPL 15 seconds to plan
  profile->planning_time = 15.0;

  // Clear existing planners
  profile->planners.clear();

  // Add an RRTConnect planner with a small step size for small motions
  auto rrt_connect_small = std::make_shared<tesseract_planning::RRTConnectConfigurator>();
  rrt_connect_small->range = 0.05;
  profile->planners.push_back(rrt_connect_small);

  // Add an RRTConnect planner with a large step size for large motions
  auto rrt_connect_large = std::make_shared<tesseract_planning::RRTConnectConfigurator>();
  rrt_connect_large->range = 0.25;
  profile->planners.push_back(rrt_connect_large);

  return profile;
}

std::shared_ptr<tesseract_planning::TrajOptPlanProfile> createTrajOptToolZFreePlanProfile()
{
  auto profile = std::make_shared<tesseract_planning::TrajOptDefaultPlanProfile>();

  profile->cartesian_coeff = Eigen::VectorXd::Constant(6, 1, 5.0);
  profile->cartesian_coeff(5) = 0.0;

  return profile;
}

std::shared_ptr<tesseract_planning::TrajOptDefaultCompositeProfile> createTrajOptProfile()
{
  auto profile = std::make_shared<tesseract_planning::TrajOptDefaultCompositeProfile>();

  profile->smooth_velocities = true;
  profile->velocity_coeff = Eigen::VectorXd::Constant(6, 1, 10.0);
  profile->acceleration_coeff = Eigen::VectorXd::Constant(6, 1, 25.0);
  profile->jerk_coeff = Eigen::VectorXd::Constant(6, 1, 50.0);

  profile->collision_cost_config.enabled = true;
  profile->collision_cost_config.type = trajopt::CollisionEvaluatorType::DISCRETE_CONTINUOUS;
  profile->collision_cost_config.safety_margin = 0.010;
  profile->collision_cost_config.safety_margin_buffer = 0.010;
  profile->collision_cost_config.coeff = 10.0;

  profile->collision_constraint_config.enabled = false;

  return profile;
}

std::shared_ptr<tesseract_planning::SimplePlannerLVSPlanProfile> createSimplePlannerProfile()
{
  return std::make_shared<tesseract_planning::SimplePlannerLVSPlanProfile>(5 * M_PI / 180, 0.1, 5 * M_PI / 180, 1);
}
