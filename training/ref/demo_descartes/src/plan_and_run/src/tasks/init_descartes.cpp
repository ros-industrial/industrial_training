#include <plan_and_run/demo_application.h>

/* INIT DESCARTES
  Goal:
    - Initialize a Descartes RobotModel object for carrying out various robot related tasks.
    - Initialize a Descartes Path Planner for planning a robot path from a trajectory.

  Hints:
    - Use the "config_" structure in order to access the application parameters
      needed to initialize the Descartes RobotModel object
*/

namespace plan_and_run
{

void DemoApplication::initDescartes()
{
  //ROS_ERROR_STREAM("Task '"<<__FUNCTION__ <<"' is incomplete. Exiting"); exit(-1);

  // Instantiating a robot model
  robot_model_ptr_.reset(new ur5_demo_descartes::UR5RobotModel());

  /*  Fill Code:
   * Goal:
   * - Initialize the "robot_model_ptr" variable by passing the required application parameters
   *    into its "initialize" method.
   * Hint:
   * - The config_ structure contains the variables needed by the robot model
   * - The "initialize" method takes the following arguments in this order
   *    a - robot description string
   *    b - group_name string.
   *    c - world_frame string
   *    d - tip_link string.
   */
/*  if(robot_model_ptr_->initialize(ROBOT_DESCRIPTION_PARAM,
                                  "[ COMPLETE HERE ]",
                                  "[ COMPLETE HERE ]",
                                  "[ COMPLETE HERE ]"))*/

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

  /*  Fill Code:
   * Goal:
   * - Initialize the Descartes path planner by calling "planner_.initialize(...)".
   * - Pass the robot_model_ptr_ created earlier into the initialize method and save the result
   *    into the "succeeded" variable.
   */
  //bool succeeded = false  /* [ COMPLETE HERE ]*/;
  bool succeeded = planner_.initialize(robot_model_ptr_);
  if(succeeded)
  {
    ROS_INFO_STREAM("Descartes Dense Planner initialized");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to initialize Dense Planner");
    exit(-1);
  }

  ROS_INFO_STREAM("Task '"<<__FUNCTION__<<"' completed");

}

}
