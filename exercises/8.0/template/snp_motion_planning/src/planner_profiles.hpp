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

  /* =======================
     * Fill Code: DESCARTES
     * =======================*/

  return profile;
}

tesseract_planning::OMPLDefaultPlanProfile::Ptr createOMPLProfile()
{
  auto profile = std::make_shared<tesseract_planning::OMPLDefaultPlanProfile>();

  /* =======================
     * Fill Code: OMPL
     * =======================*/

  return profile;
}

std::shared_ptr<tesseract_planning::TrajOptPlanProfile> createTrajOptToolZFreePlanProfile()
{
  auto profile = std::make_shared<tesseract_planning::TrajOptDefaultPlanProfile>();

  /* =======================
     * Fill Code: TRAJOPT PLAN
     * =======================*/

  return profile;
}

std::shared_ptr<tesseract_planning::TrajOptDefaultCompositeProfile> createTrajOptProfile()
{
  auto profile = std::make_shared<tesseract_planning::TrajOptDefaultCompositeProfile>();

  /* =======================
     * Fill Code: TRAJOPT COMPOSITE
     * =======================*/

  return profile;
}

std::shared_ptr<tesseract_planning::SimplePlannerLVSPlanProfile> createSimplePlannerProfile()
{
  return std::make_shared<tesseract_planning::SimplePlannerLVSPlanProfile>(5 * M_PI / 180, 0.1, 5 * M_PI / 180, 1);
}
