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
  /* =======================
   * Fill Code: DESCARTES 
   * =======================*/
   return std::make_shared<tesseract_planning::DescartesDefaultPlanProfile<float>>();
}

tesseract_planning::OMPLDefaultPlanProfile::Ptr createOMPLProfile()
{
  /* =======================
   * Fill Code: OMPL 
   * =======================*/
    return std::make_shared<tesseract_planning::OMPLDefaultPlanProfile>();
}

std::shared_ptr<tesseract_planning::TrajOptPlanProfile> createTrajOptToolZFreePlanProfile()
{
  /* ==========================
   * Fill Code: TRAJOPT PLAN 
   * ==========================*/
   return std::make_shared<tesseract_planning::TrajOptDefaultPlanProfile>();
}

std::shared_ptr<tesseract_planning::TrajOptDefaultCompositeProfile> createTrajOptProfile()
{
  /* ==============================
   * Fill Code: TRAJOPT COMPOSITE 
   * ==============================*/
   return std::make_shared<tesseract_planning::TrajOptDefaultCompositeProfile>();
}

std::shared_ptr<tesseract_planning::SimplePlannerLVSPlanProfile> createSimplePlannerProfile()
{
  return std::make_shared<tesseract_planning::SimplePlannerLVSPlanProfile>(5 * M_PI / 180, 0.1, 5 * M_PI / 180, 5);
}
