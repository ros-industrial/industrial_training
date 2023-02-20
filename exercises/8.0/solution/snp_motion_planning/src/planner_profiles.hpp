#pragma once

#include <descartes_light/edge_evaluators/compound_edge_evaluator.h>
#include <descartes_light/edge_evaluators/euclidean_distance_edge_evaluator.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>
#include <tesseract_motion_planners/ompl/profile/ompl_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_plan_profile.h>

template <typename FloatType>
typename tesseract_planning::DescartesDefaultPlanProfile<FloatType>::Ptr createDescartesPlanProfile()
{
  auto profile = std::make_shared<tesseract_planning::DescartesDefaultPlanProfile<FloatType>>();
  profile->num_threads = static_cast<int>(std::thread::hardware_concurrency());
  profile->use_redundant_joint_solutions = false;
  profile->allow_collision = false;
  profile->enable_collision = true;
  profile->enable_edge_collision = false;

  // Use the default state and edge evaluators
  profile->state_evaluator = nullptr;
  profile->edge_evaluator = [](const tesseract_planning::DescartesProblem<FloatType> & /*prob*/) ->
      typename descartes_light::EdgeEvaluator<FloatType>::Ptr {
        auto eval = std::make_shared<descartes_light::CompoundEdgeEvaluator<FloatType>>();

        // Nominal Euclidean distance
        eval->evaluators.push_back(std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluator<FloatType>>());

        // Penalize wrist motion
        //        Eigen::Matrix<FloatType, Eigen::Dynamic, 1> wrist_mask(prob.manip->numJoints());
        //        FloatType weight = static_cast<FloatType>(5.0);
        //        wrist_mask << 0.0, 0.0, 0.0, weight, weight, weight;
        //        eval->evaluators.push_back(
        //            std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluator<FloatType>>(wrist_mask));

        return eval;
      };

  profile->vertex_evaluator = nullptr;

  profile->target_pose_sampler =
      std::bind(tesseract_planning::sampleToolZAxis, std::placeholders::_1, 10.0 * M_PI / 180.0);

  return profile;
}

tesseract_planning::OMPLDefaultPlanProfile::Ptr createOMPLProfile()
{
  // OMPL freespace and transition profiles
  // Create the RRT parameters
  auto n = static_cast<Eigen::Index>(std::thread::hardware_concurrency());
  auto range = Eigen::VectorXd::LinSpaced(n, 0.05, 0.5);

  // Add as many planners as available threads so mulitple OMPL plans can happen in parallel
  auto profile = std::make_shared<tesseract_planning::OMPLDefaultPlanProfile>();
  profile->planning_time = 20.0;
  profile->planners.clear();
  profile->planners.reserve(static_cast<std::size_t>(n));
  for (Eigen::Index i = 0; i < n; ++i)
  {
    auto rrt = std::make_shared<tesseract_planning::RRTConnectConfigurator>();
    rrt->range = range(i);
    profile->planners.push_back(rrt);
  }

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
  // TrajOpt profiles
  auto profile = std::make_shared<tesseract_planning::TrajOptDefaultCompositeProfile>();
  profile->smooth_velocities = true;

  profile->smooth_accelerations = true;
  profile->smooth_jerks = false;
  profile->acceleration_coeff = Eigen::VectorXd::Constant(6, 1, 10.0);
  profile->jerk_coeff = Eigen::VectorXd::Constant(6, 1, 20.0);

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
  return std::make_shared<tesseract_planning::SimplePlannerLVSPlanProfile>(5 * M_PI / 180, 0.1, 5 * M_PI / 180, 5);
}
