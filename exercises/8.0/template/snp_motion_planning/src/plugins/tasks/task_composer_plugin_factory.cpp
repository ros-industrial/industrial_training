#include <tesseract_task_composer/task_composer_task_plugin_factory.h>
#include <tesseract_task_composer/task_composer_graph.h>

#include "constant_tcp_speed_time_parameterization_task.hpp"

namespace snp_motion_planning
{
using ConstantTCPSpeedTimeParameterizationTaskFactory =
    tesseract_planning::TaskComposerTaskFactory<ConstantTCPSpeedTimeParameterizationTask>;
}  // namespace snp_motion_planning

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(snp_motion_planning::ConstantTCPSpeedTimeParameterizationTaskFactory,
                                        ConstantTCPSpeedTimeParameterizationTaskFactory)
