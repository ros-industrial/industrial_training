#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
//#include <lesson_actions/FibonacciAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_fibonacci");

  // create action client, true causes the client to spin its own thread
//  actionlib::SimpleActionClient<lesson_actions::FibonacciAction> ac("fibonacci", true);

  ROS_INFO("Waiting for action server to start.");
//  ac.waitForServer(); //will wait (for infinite time) for server to start

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
//  lesson_actions::FibonacciGoal goal;
//  goal.order = 20;
//  ac.sendGoal(goal);

  //wait for the action to return
//  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
//    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  return 0;
}
