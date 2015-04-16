#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <lesson_actions/CalcPiAction.h>

typedef actionlib::SimpleActionClient<lesson_actions::CalcPiAction> ActionClient;

//-----------------------------------------------------------------------------
// Define callback functions
//-----------------------------------------------------------------------------
void activeCb()
{
  ROS_INFO("Action active");
}

void doneCb(const actionlib::SimpleClientGoalState& state,
            const lesson_actions::CalcPiResultConstPtr& result)
{
    ROS_INFO_STREAM("Action finished: " << state.toString());
    ROS_INFO_STREAM("Pi estimate: " << result->pi);
}

void feedbackCb(const lesson_actions::CalcPiFeedbackConstPtr& feedback)
{
  ROS_INFO_STREAM("Pi estimate: " << feedback->pi << " (" << std::setprecision(1) << std::scientific << (double)feedback->iter << " iter)");
}
//-----------------------------------------------------------------------------

int main (int argc, char **argv)
{
  ros::init(argc, argv, "calcPi_clientNode");

  // create action client
  ActionClient ac("calcPiAction");

  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer(); //will wait (for infinite time) for server to start

  ROS_INFO("Action server started, sending goal.");
  lesson_actions::CalcPiGoal goal;  // send a goal to the action
  goal.digits =  (argc > 1) ? atoi(argv[1]) : 7;
  ac.sendGoal(goal, doneCb, activeCb, feedbackCb);

  // wait for result
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (!finished_before_timeout)
    ROS_INFO("Action did not finish before the time out.");

  return 0;
}
