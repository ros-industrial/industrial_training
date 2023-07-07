#pragma once

#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/simple/simple_motion_planner.h>
#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
// Task Composer
#include <tesseract_task_composer/task_composer_problem.h>
#include <tesseract_task_composer/task_composer_input.h>
#include <tesseract_task_composer/task_composer_node_names.h>
#include <tesseract_task_composer/taskflow/taskflow_task_composer_executor.h>
#include <tesseract_task_composer/profiles/contact_check_profile.h>

#include <tesseract_common/timer.h>

#include <tesseract_task_composer/nodes/min_length_task.h>
#include <tesseract_task_composer/nodes/discrete_contact_check_task.h>
#include <tesseract_task_composer/nodes/iterative_spline_parameterization_task.h>
#include <tesseract_task_composer/nodes/done_task.h>
#include <tesseract_task_composer/nodes/error_task.h>

#include <tesseract_task_composer/nodes/ompl_motion_planner_task.h>
#include <tesseract_task_composer/nodes/trajopt_motion_planner_task.h>
#include <tesseract_task_composer/nodes/simple_motion_planner_task.h>
#include <tesseract_task_composer/nodes/descartes_motion_planner_task.h>

#include <tesseract_task_composer/nodes/descartes_global_motion_pipeline_task.h>
#include <tesseract_task_composer/nodes/simple_motion_pipeline_task.h>
#include <tesseract_task_composer/nodes/raster_motion_task.hpp>
#include <tesseract_task_composer/nodes/raster_global_pipeline_task.hpp>

namespace snp_planning
{
class FreespaceMotionPipelineTask : public tesseract_planning::TaskComposerGraph
{
public:
  using Ptr = std::shared_ptr<FreespaceMotionPipelineTask>;
  using ConstPtr = std::shared_ptr<const FreespaceMotionPipelineTask>;
  using UPtr = std::unique_ptr<FreespaceMotionPipelineTask>;
  using ConstUPtr = std::unique_ptr<const FreespaceMotionPipelineTask>;

  /**
   * @brief FreespaceMotionPipelineTask
   * @details This will use the uuid as the input and output key
   * @param name The name give to the task
   */
  FreespaceMotionPipelineTask(std::string name = tesseract_planning::node_names::FREESPACE_PIPELINE_NAME)
    : tesseract_planning::TaskComposerGraph(std::move(name))
  {
    ctor(uuid_str_, uuid_str_);
  }

  FreespaceMotionPipelineTask(std::string input_key, std::string output_key,
                              std::string name = tesseract_planning::node_names::FREESPACE_PIPELINE_NAME)
    : tesseract_planning::TaskComposerGraph(std::move(name))
  {
    ctor(std::move(input_key), std::move(output_key));
  }

  ~FreespaceMotionPipelineTask() override = default;
  FreespaceMotionPipelineTask(const FreespaceMotionPipelineTask&) = delete;
  FreespaceMotionPipelineTask& operator=(const FreespaceMotionPipelineTask&) = delete;
  FreespaceMotionPipelineTask(FreespaceMotionPipelineTask&&) = delete;
  FreespaceMotionPipelineTask& operator=(FreespaceMotionPipelineTask&&) = delete;

  bool operator==(const FreespaceMotionPipelineTask& rhs) const
  {
    bool equal = true;
    equal &= TaskComposerGraph::operator==(rhs);
    return equal;
  }

  bool operator!=(const FreespaceMotionPipelineTask& rhs) const
  {
    return !operator==(rhs);
  }

protected:
  friend class tesseract_common::Serialization;
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerGraph);
  }

  void ctor(std::string input_key, std::string output_key)
  {
    input_keys_.push_back(std::move(input_key));
    output_keys_.push_back(std::move(output_key));

    boost::uuids::uuid done_task = addNode(std::make_unique<tesseract_planning::DoneTask>());
    boost::uuids::uuid error_task = addNode(std::make_unique<tesseract_planning::ErrorTask>());

    // Setup Min Length Process Generator
    // This is required because trajopt requires a minimum length trajectory.
    // This is used to correct the input if it is to short.
    boost::uuids::uuid min_length_task =
        addNode(std::make_unique<tesseract_planning::MinLengthTask>(input_keys_[0], output_keys_[0]));

    /* ========================================
     * Fill Code: CREATE CUSTOM PLANNER NODES
     * ========================================*/

    // Setup post collision check
    boost::uuids::uuid contact_check_task =
        addNode(std::make_unique<tesseract_planning::DiscreteContactCheckTask>(output_keys_[0]));

    // Setup time parameterization
    boost::uuids::uuid time_parameterization_task = addNode(
        std::make_unique<tesseract_planning::IterativeSplineParameterizationTask>(output_keys_[0], output_keys_[0]));

    /* =======================
     * Fill Code: EDIT EDGES
     * =======================*/
    addEdges(min_length_task, { contact_check_task });
    addEdges(contact_check_task, { error_task, time_parameterization_task });
    addEdges(time_parameterization_task, { error_task, done_task });
  }
};

class TransitionMotionPipelineTask : public tesseract_planning::TaskComposerGraph
{
public:
  using Ptr = std::shared_ptr<TransitionMotionPipelineTask>;
  using ConstPtr = std::shared_ptr<const TransitionMotionPipelineTask>;
  using UPtr = std::unique_ptr<TransitionMotionPipelineTask>;
  using ConstUPtr = std::unique_ptr<const TransitionMotionPipelineTask>;

  /**
   * @brief TransitionMotionPipelineTask
   * @details This will use the uuid as the input and output key
   * @param name The name give to the task
   */
  TransitionMotionPipelineTask(std::string name = tesseract_planning::node_names::FREESPACE_PIPELINE_NAME)
    : tesseract_planning::TaskComposerGraph(std::move(name))
  {
    ctor(uuid_str_, uuid_str_);
  }

  TransitionMotionPipelineTask(std::string input_key, std::string output_key,
                               std::string name = tesseract_planning::node_names::FREESPACE_PIPELINE_NAME)
    : tesseract_planning::TaskComposerGraph(std::move(name))
  {
    ctor(std::move(input_key), std::move(output_key));
  }

  ~TransitionMotionPipelineTask() override = default;
  TransitionMotionPipelineTask(const TransitionMotionPipelineTask&) = delete;
  TransitionMotionPipelineTask& operator=(const TransitionMotionPipelineTask&) = delete;
  TransitionMotionPipelineTask(TransitionMotionPipelineTask&&) = delete;
  TransitionMotionPipelineTask& operator=(TransitionMotionPipelineTask&&) = delete;

  bool operator==(const TransitionMotionPipelineTask& rhs) const
  {
    bool equal = true;
    equal &= TaskComposerGraph::operator==(rhs);
    return equal;
  }

  bool operator!=(const TransitionMotionPipelineTask& rhs) const
  {
    return !operator==(rhs);
  }

protected:
  friend class tesseract_common::Serialization;
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerGraph);
  }

  void ctor(std::string input_key, std::string output_key)
  {
    input_keys_.push_back(std::move(input_key));
    output_keys_.push_back(std::move(output_key));

    boost::uuids::uuid done_task = addNode(std::make_unique<tesseract_planning::DoneTask>());
    boost::uuids::uuid error_task = addNode(std::make_unique<tesseract_planning::ErrorTask>());

    // Setup Min Length Process Generator
    // This is required because trajopt requires a minimum length trajectory.
    // This is used to correct the input if it is to short.
    boost::uuids::uuid min_length_task =
        addNode(std::make_unique<tesseract_planning::MinLengthTask>(input_keys_[0], output_keys_[0]));

    // Simple planner
    boost::uuids::uuid simple_task =
        addNode(std::make_unique<tesseract_planning::SimpleMotionPlannerTask>(input_keys_[0], output_keys_[0], false));

    /* ========================================
     * Fill Code: CREATE CUSTOM PLANNER NODES
     * ========================================*/

    // Setup post collision check
    boost::uuids::uuid contact_check_task =
        addNode(std::make_unique<tesseract_planning::DiscreteContactCheckTask>(output_keys_[0]));

    // Setup time parameterization
    boost::uuids::uuid time_parameterization_task = addNode(
        std::make_unique<tesseract_planning::IterativeSplineParameterizationTask>(output_keys_[0], output_keys_[0]));

    /* =======================
     * Fill Code: EDIT EDGES
     * =======================*/
    addEdges(simple_task, { min_length_task });
    addEdges(min_length_task, { contact_check_task });
    addEdges(contact_check_task, { error_task, time_parameterization_task });
    addEdges(time_parameterization_task, { error_task, done_task });
  }
};

class CartesianMotionPipelineTask : public tesseract_planning::TaskComposerGraph
{
public:
  using Ptr = std::shared_ptr<CartesianMotionPipelineTask>;
  using ConstPtr = std::shared_ptr<const CartesianMotionPipelineTask>;
  using UPtr = std::unique_ptr<CartesianMotionPipelineTask>;
  using ConstUPtr = std::unique_ptr<const CartesianMotionPipelineTask>;

  /**
   * @brief CartesianMotionPipelineTask
   * @details This will use the uuid as the input and output key
   * @param name The name give to the task
   */
  CartesianMotionPipelineTask(std::string name = tesseract_planning::node_names::FREESPACE_PIPELINE_NAME)
    : tesseract_planning::TaskComposerGraph(std::move(name))
  {
    ctor(uuid_str_, uuid_str_);
  }

  CartesianMotionPipelineTask(std::string input_key, std::string output_key,
                              std::string name = tesseract_planning::node_names::FREESPACE_PIPELINE_NAME)
    : tesseract_planning::TaskComposerGraph(std::move(name))
  {
    ctor(std::move(input_key), std::move(output_key));
  }

  ~CartesianMotionPipelineTask() override = default;
  CartesianMotionPipelineTask(const CartesianMotionPipelineTask&) = delete;
  CartesianMotionPipelineTask& operator=(const CartesianMotionPipelineTask&) = delete;
  CartesianMotionPipelineTask(CartesianMotionPipelineTask&&) = delete;
  CartesianMotionPipelineTask& operator=(CartesianMotionPipelineTask&&) = delete;

  bool operator==(const CartesianMotionPipelineTask& rhs) const
  {
    bool equal = true;
    equal &= TaskComposerGraph::operator==(rhs);
    return equal;
  }

  bool operator!=(const CartesianMotionPipelineTask& rhs) const
  {
    return !operator==(rhs);
  }

protected:
  friend class tesseract_common::Serialization;
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerGraph);
  }

  void ctor(std::string input_key, std::string output_key)
  {
    input_keys_.push_back(std::move(input_key));
    output_keys_.push_back(std::move(output_key));

    boost::uuids::uuid done_task = addNode(std::make_unique<tesseract_planning::DoneTask>());
    boost::uuids::uuid error_task = addNode(std::make_unique<tesseract_planning::ErrorTask>());

    // Setup Min Length ProcessDescartes Generator
    // This is required because trajopt requires a minimum length trajectory.
    // This is used to correct the input if it is to short.
    boost::uuids::uuid min_length_task =
        addNode(std::make_unique<tesseract_planning::MinLengthTask>(input_keys_[0], output_keys_[0]));

    /* ========================================
     * Fill Code: CREATE CUSTOM PLANNER NODES
     * ========================================*/

    // Setup post collision check
    boost::uuids::uuid contact_check_task =
        addNode(std::make_unique<tesseract_planning::DiscreteContactCheckTask>(output_keys_[0]));

    // Setup time parameterization
    boost::uuids::uuid time_parameterization_task = addNode(
        std::make_unique<tesseract_planning::IterativeSplineParameterizationTask>(output_keys_[0], output_keys_[0]));

    /* =======================
     * Fill Code: EDIT EDGES
     * =======================*/
    addEdges(min_length_task, { contact_check_task });
    addEdges(contact_check_task, { error_task, time_parameterization_task });
    addEdges(time_parameterization_task, { error_task, done_task });
  }
};

}  // namespace snp_planning

using CustomRasterPipeline = tesseract_planning::RasterMotionTask<snp_planning::FreespaceMotionPipelineTask,
                                                                  snp_planning::CartesianMotionPipelineTask,
                                                                  snp_planning::TransitionMotionPipelineTask>;
using CustomGlobalRasterGlobalPipeline =
          tesseract_planning::RasterGlobalPipelineTask<tesseract_planning::SimpleMotionPipelineTask,
                                                       tesseract_planning::DescartesGlobalMotionPipelineTask,
                                                       CustomRasterPipeline>;


CustomGlobalRasterGlobalPipeline::UPtr createGlobalRasterPipeline()
{
  CustomGlobalRasterGlobalPipeline::UPtr task =
      std::make_unique<CustomGlobalRasterGlobalPipeline>("input_program", "output_program", "custom_global_pipeline");

  // Write the graph to the tmp directory for debugging
  std::ofstream tc_out_data;
  tc_out_data.open(tesseract_common::getTempPath() + "global_task_graph.dot");
  task->dump(tc_out_data);  // dump the graph including dynamic tasks
  tc_out_data.close();

  return task;
}

#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(snp_planning::FreespaceMotionPipelineTask, "FreespaceMotionPipelineTask")
BOOST_CLASS_EXPORT_KEY2(snp_planning::TransitionMotionPipelineTask, "TransitionMotionPipelineTask")
BOOST_CLASS_EXPORT_KEY2(snp_planning::CartesianMotionPipelineTask, "CartesianMotionPipelineTask")
