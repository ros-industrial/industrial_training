#include <plan_and_run/demo_application.h>

/* INIT ROS
  Goal:
    - Create a ros service client that will be used to send a robot path for execution.

  Hints:
*/

namespace plan_and_run
{

void DemoApplication::initRos()
{
  //ROS_ERROR_STREAM("Task '"<<__FUNCTION__ <<"' is incomplete. Exiting"); exit(-1);

  // creating publisher for trajectory visualization
  marker_publisher_  = nh_.advertise<visualization_msgs::MarkerArray>(VISUALIZE_TRAJECTORY_TOPIC,1,true);

  /*  Fill Code:
   * Goal:
   * - Create a "moveit_msgs::ExecuteTrajectoryAction" client and assign it to the "moveit_run_path_client_ptr_"
   *    application variable.
   * - Uncomment the action client initialization code.
   * Hint:
   * - Enter the action type moveit_msgs::ExecuteTrajectoryAction in between the "< >" arrow brackets of
   *   the client type definition, replacing the placeholder type.
   */
  typedef actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction> client_type;
  moveit_run_path_client_ptr_ = std::make_shared<client_type>(EXECUTE_TRAJECTORY_ACTION,true);

  // Establishing connection to server
  if(moveit_run_path_client_ptr_->waitForServer(ros::Duration(SERVER_TIMEOUT)))
  {
    ROS_INFO_STREAM("Connected to '"<<EXECUTE_TRAJECTORY_ACTION<<"' action");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to connect to '"<<EXECUTE_TRAJECTORY_ACTION<<"' action");
    exit(-1);
  }

  ROS_INFO_STREAM("Task '"<<__FUNCTION__<<"' completed");

}

}

