#include "constant_tcp_speed_time_parameterization.hpp"
#include "constant_tcp_speed_time_parameterization_profile.hpp"

#include <tesseract_task_composer/core/task_composer_plugin_factory_utils.h>

#include <tesseract_common/macros.h>
#include <console_bridge/console.h>
#include <boost/serialization/string.hpp>

#include <tesseract_common/timer.h>
#include <tesseract_motion_planners/planner_utils.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_time_parameterization/core/instructions_trajectory.h>

#include <tesseract_task_composer/core/task_composer_task.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory_utils.h>
#include <tesseract_task_composer/planning/planning_task_composer_problem.h>

namespace snp_motion_planning
{
class ConstantTCPSpeedTimeParameterizationTask : public tesseract_planning::TaskComposerTask
{
public:
  using Ptr = std::shared_ptr<ConstantTCPSpeedTimeParameterizationTask>;
  using ConstPtr = std::shared_ptr<const ConstantTCPSpeedTimeParameterizationTask>;
  using UPtr = std::unique_ptr<ConstantTCPSpeedTimeParameterizationTask>;
  using ConstUPtr = std::unique_ptr<const ConstantTCPSpeedTimeParameterizationTask>;

  ConstantTCPSpeedTimeParameterizationTask()
    : tesseract_planning::TaskComposerTask(CONSTANT_TCP_SPEED_TIME_PARAM_TASK_NAME, true)
  {
  }

  explicit ConstantTCPSpeedTimeParameterizationTask(std::string name, std::string input_key, std::string output_key,
                                                    bool is_conditional = true)
    : tesseract_planning::TaskComposerTask(std::move(name), is_conditional)
  {
    input_keys_.push_back(std::move(input_key));
    output_keys_.push_back(std::move(output_key));
  }

  explicit ConstantTCPSpeedTimeParameterizationTask(
      std::string name, const YAML::Node& config,
      const tesseract_planning::TaskComposerPluginFactory& /*plugin_factory*/)
    : tesseract_planning::TaskComposerTask(std::move(name), config)
  {
    if (input_keys_.empty())
      throw std::runtime_error("ConstantTCPSpeedTimeParameterizationTask, config missing 'inputs' entry");

    if (input_keys_.size() > 1)
      throw std::runtime_error("ConstantTCPSpeedTimeParameterizationTask, config 'inputs' entry currently only "
                               "supports "
                               "one input key");

    if (output_keys_.empty())
      throw std::runtime_error("ConstantTCPSpeedTimeParameterizationTask, config missing 'outputs' entry");

    if (output_keys_.size() > 1)
      throw std::runtime_error("ConstantTCPSpeedTimeParameterizationTask, config 'outputs' entry currently only "
                               "supports "
                               "one output key");
  }

  ~ConstantTCPSpeedTimeParameterizationTask() override = default;
  ConstantTCPSpeedTimeParameterizationTask(const ConstantTCPSpeedTimeParameterizationTask&) = delete;
  ConstantTCPSpeedTimeParameterizationTask& operator=(const ConstantTCPSpeedTimeParameterizationTask&) = delete;
  ConstantTCPSpeedTimeParameterizationTask(ConstantTCPSpeedTimeParameterization&&) = delete;
  ConstantTCPSpeedTimeParameterizationTask& operator=(ConstantTCPSpeedTimeParameterizationTask&&) = delete;

  bool operator==(const ConstantTCPSpeedTimeParameterizationTask& rhs) const
  {
    bool equal = true;
    equal &= tesseract_planning::TaskComposerTask::operator==(rhs);
    return equal;
  }
  bool operator!=(const ConstantTCPSpeedTimeParameterizationTask& rhs) const
  {
    return !operator==(rhs);
  }

protected:
  friend struct tesseract_common::Serialization;
  friend class boost::serialization::access;
  tesseract_planning::TaskComposerNodeInfo::UPtr runImpl(tesseract_planning::TaskComposerInput& input,
                                                         OptionalTaskComposerExecutor /*executor*/) const override final
  {
    auto info = std::make_unique<tesseract_planning::TaskComposerNodeInfo>(*this);
    info->return_value = 0;

    if (input.isAborted())
    {
      info->message = "Aborted";
      return info;
    }

    // Get the problem
    auto& problem = dynamic_cast<tesseract_planning::PlanningTaskComposerProblem&>(*input.problem);

    tesseract_common::Timer timer;
    timer.start();

    // --------------------
    // Check that inputs are valid
    // --------------------
    auto input_data_poly = input.data_storage.getData(input_keys_[0]);
    if (input_data_poly.isNull() ||
        input_data_poly.getType() != std::type_index(typeid(tesseract_planning::CompositeInstruction)))
    {
      info->message = "Input results to constant TCP speed time parameterization must be a composite instruction";
      info->elapsed_time = timer.elapsedSeconds();
      CONSOLE_BRIDGE_logError("%s", info->message.c_str());
      return info;
    }

    auto& ci = input_data_poly.as<tesseract_planning::CompositeInstruction>();
    const tesseract_common::ManipulatorInfo& manip_info = ci.getManipulatorInfo();

    // Get Composite Profile
    std::string profile = ci.getProfile();
    profile = tesseract_planning::getProfileString(name_, profile, problem.composite_profile_remapping);
    auto cur_composite_profile = tesseract_planning::getProfile<ConstantTCPSpeedTimeParameterizationProfile>(
        name_, profile, *problem.profiles, std::make_shared<ConstantTCPSpeedTimeParameterizationProfile>());
    cur_composite_profile = applyProfileOverrides(name_, profile, cur_composite_profile, ci.getProfileOverrides());

    // Create data structures for checking for plan profile overrides
    auto flattened = ci.flatten(tesseract_planning::moveFilter);
    if (flattened.empty())
    {
      info->message = "Cartesian time parameterization found no MoveInstructions to process";
      info->return_value = 1;
      info->elapsed_time = timer.elapsedSeconds();
      return info;
    }

    // Solve using parameters
    tesseract_planning::TrajectoryContainer::Ptr trajectory =
        std::make_shared<tesseract_planning::InstructionsTrajectory>(ci);

    ConstantTCPSpeedTimeParameterization solver(
        problem.env, manip_info.manipulator, manip_info.tcp_frame, cur_composite_profile->max_translational_velocity,
        cur_composite_profile->max_rotational_velocity, cur_composite_profile->max_translational_acceleration,
        cur_composite_profile->max_rotational_acceleration);

    if (!solver.compute(*trajectory, cur_composite_profile->max_velocity_scaling_factor,
                        cur_composite_profile->max_acceleration_scaling_factor))
    {
      info->message =
          "Failed to perform constant TCP speed time parameterization for process input: " + ci.getDescription();
      info->elapsed_time = timer.elapsedSeconds();
      return info;
    }

    info->color = "green";
    info->message = "Constant TCP speed time parameterization succeeded";
    input.data_storage.setData(output_keys_[0], input_data_poly);
    info->return_value = 1;
    info->elapsed_time = timer.elapsedSeconds();
    return info;
  }

  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/)
  {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(tesseract_planning::TaskComposerTask);
  }
};

}  // namespace snp_motion_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(snp_motion_planning::ConstantTCPSpeedTimeParameterizationTask)
BOOST_CLASS_EXPORT_IMPLEMENT(snp_motion_planning::ConstantTCPSpeedTimeParameterizationTask)

TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(
    tesseract_planning::TaskComposerTaskFactory<snp_motion_planning::ConstantTCPSpeedTimeParameterizationTask>,
    ConstantTCPSpeedTimeParameterizationTaskFactory)
