#pragma once

#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/simple/simple_motion_planner.h>
#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
// Task Generators
#include <tesseract_process_managers/task_generators/check_input_task_generator.h>
#include <tesseract_process_managers/task_generators/has_seed_task_generator.h>
#include <tesseract_process_managers/task_generators/seed_min_length_task_generator.h>
#include <tesseract_process_managers/task_generators/motion_planner_task_generator.h>
#include <tesseract_process_managers/task_generators/discrete_contact_check_task_generator.h>
#include <tesseract_process_managers/task_generators/iterative_spline_parameterization_task_generator.h>
// Taskflow generators
#include <tesseract_process_managers/taskflow_generators/graph_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/raster_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/raster_global_taskflow.h>
#include <tesseract_process_managers/core/default_process_planners.h>

/**
 * @brief Creates a task flow graph for planning transition moves using a simple planner and TrajOpt with time
 * parameterization
 * @return
 */
tesseract_planning::TaskflowGenerator::UPtr createTransitionTaskflow()
{
  // Create the graph task flow
  auto graph = std::make_unique<tesseract_planning::GraphTaskflow>();

  // Input/seed checks
  auto check_input = graph->addNode(std::make_unique<tesseract_planning::CheckInputTaskGenerator>(), true);
  int has_seed = graph->addNode(std::make_unique<tesseract_planning::HasSeedTaskGenerator>(), true);
  int seed_min_length = graph->addNode(std::make_unique<tesseract_planning::SeedMinLengthTaskGenerator>(), true);

  // Simple planner with post-collision check
  auto simple_planner = std::make_shared<tesseract_planning::SimpleMotionPlanner>();
  int simple = graph->addNode(std::make_unique<tesseract_planning::MotionPlannerTaskGenerator>(simple_planner), true);
  int simple_collision =
      graph->addNode(std::make_unique<tesseract_planning::DiscreteContactCheckTaskGenerator>(), true);

  // TrajOpt with post-collision check
  auto trajopt_planner = std::make_shared<tesseract_planning::TrajOptMotionPlanner>();
  int trajopt = graph->addNode(std::make_unique<tesseract_planning::MotionPlannerTaskGenerator>(trajopt_planner), true);
  int trajopt_collision =
      graph->addNode(std::make_unique<tesseract_planning::DiscreteContactCheckTaskGenerator>(), true);

  // Time parameterization
  int time_param =
      graph->addNode(std::make_unique<tesseract_planning::IterativeSplineParameterizationTaskGenerator>(), true);

  graph->addEdges(check_input, { tesseract_planning::GraphTaskflow::ERROR_NODE, has_seed });
  graph->addEdges(has_seed, { simple, seed_min_length });
  graph->addEdges(seed_min_length, { tesseract_planning::GraphTaskflow::ERROR_NODE, simple_collision });
  graph->addEdges(simple, { tesseract_planning::GraphTaskflow::ERROR_NODE, seed_min_length });
  graph->addEdges(simple_collision, { trajopt, time_param });
  graph->addEdges(trajopt, { tesseract_planning::GraphTaskflow::ERROR_NODE, trajopt_collision });
  graph->addEdges(trajopt_collision, { tesseract_planning::GraphTaskflow::ERROR_NODE, time_param });
  graph->addEdges(time_param,
                  { tesseract_planning::GraphTaskflow::ERROR_NODE, tesseract_planning::GraphTaskflow::DONE_NODE });

  return graph;
}

/**
 * @brief Creates a task flow graph for planning freespace motions using a simple planner, TrajOpt, and OMPL smoothed by
 * TrajOpt, with time parameterization
 * @return
 */
tesseract_planning::TaskflowGenerator::UPtr createFreespaceTaskflow()
{
  // Create the graph task flow
  auto graph = std::make_unique<tesseract_planning::GraphTaskflow>();

  // Input/seed checks
  auto check_input = graph->addNode(std::make_unique<tesseract_planning::CheckInputTaskGenerator>(), true);
  int has_seed = graph->addNode(std::make_unique<tesseract_planning::HasSeedTaskGenerator>(), true);
  int seed_min_length = graph->addNode(std::make_unique<tesseract_planning::SeedMinLengthTaskGenerator>(), true);

  // Simple planner with post-collision check
  auto simple_planner = std::make_shared<tesseract_planning::SimpleMotionPlanner>();
  int simple = graph->addNode(std::make_unique<tesseract_planning::MotionPlannerTaskGenerator>(simple_planner), true);
  int simple_collision =
      graph->addNode(std::make_unique<tesseract_planning::DiscreteContactCheckTaskGenerator>(), true);

  // TrajOpt with post-collision check
  auto trajopt_planner = std::make_shared<tesseract_planning::TrajOptMotionPlanner>();
  int trajopt = graph->addNode(std::make_unique<tesseract_planning::MotionPlannerTaskGenerator>(trajopt_planner), true);
  int trajopt_collision =
      graph->addNode(std::make_unique<tesseract_planning::DiscreteContactCheckTaskGenerator>(), true);

  // OMPL smoothed by TrajOpt with post-plan collision check
  auto ompl_planner = std::make_shared<tesseract_planning::OMPLMotionPlanner>();
  int ompl = graph->addNode(std::make_unique<tesseract_planning::MotionPlannerTaskGenerator>(ompl_planner), true);
  auto ompl_trajopt_planner = std::make_shared<tesseract_planning::TrajOptMotionPlanner>();
  int ompl_trajopt =
      graph->addNode(std::make_unique<tesseract_planning::MotionPlannerTaskGenerator>(ompl_trajopt_planner), true);
  int ompl_collision = graph->addNode(std::make_unique<tesseract_planning::DiscreteContactCheckTaskGenerator>(), true);

  // Time parameterization
  int time_param =
      graph->addNode(std::make_unique<tesseract_planning::IterativeSplineParameterizationTaskGenerator>(), true);

  graph->addEdges(check_input, { tesseract_planning::GraphTaskflow::ERROR_NODE, has_seed });
  graph->addEdges(has_seed, { simple, seed_min_length });
  graph->addEdges(seed_min_length, { tesseract_planning::GraphTaskflow::ERROR_NODE, simple_collision });
  graph->addEdges(simple, { tesseract_planning::GraphTaskflow::ERROR_NODE, seed_min_length });
  graph->addEdges(simple_collision, { trajopt, time_param });
  graph->addEdges(trajopt, { ompl, trajopt_collision });
  graph->addEdges(trajopt_collision, { ompl, time_param });
  graph->addEdges(ompl, { tesseract_planning::GraphTaskflow::ERROR_NODE, ompl_trajopt });
  graph->addEdges(ompl_trajopt, { tesseract_planning::GraphTaskflow::ERROR_NODE, ompl_collision });
  graph->addEdges(ompl_collision, { tesseract_planning::GraphTaskflow::ERROR_NODE, time_param });
  graph->addEdges(time_param,
                  { tesseract_planning::GraphTaskflow::ERROR_NODE, tesseract_planning::GraphTaskflow::DONE_NODE });

  return graph;
}

/**
 * @brief Creates a raster taskflow using the custom-defined freespace and transition planning taskflows
 */
tesseract_planning::TaskflowGenerator::UPtr createRasterTaskflow()
{
  return std::make_unique<tesseract_planning::RasterGlobalTaskflow>(
      tesseract_planning::createDescartesOnlyGenerator(), createFreespaceTaskflow(), createTransitionTaskflow(),
      tesseract_planning::createCartesianGenerator());
}
