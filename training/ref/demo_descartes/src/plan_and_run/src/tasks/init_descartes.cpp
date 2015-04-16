#include <plan_and_run/demo_application.h>

namespace plan_and_run
{

void DemoApplication::initDescartes()
{
  // Robot Model initialization
  robot_model_ptr_.reset(new ur5_demo_descartes::UR5RobotModel());
  if(robot_model_ptr_->initialize(ROBOT_DESCRIPTION_PARAM,
                                  config_.group_name,
                                  config_.world_frame,
                                  config_.tip_link))
  {
    ROS_INFO_STREAM("Descartes Robot Model initialized");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to initialize Robot Model");
    exit(-1);
  }

  // Planner initialization
  if(planner_.initialize(robot_model_ptr_))
  {
    ROS_INFO_STREAM("Descartes Dense Planner initialized");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to initialize Dense Planner");
    exit(-1);
  }

}

}
