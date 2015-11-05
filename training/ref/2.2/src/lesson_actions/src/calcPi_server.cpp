#include <ros/ros.h>
#include <calcPi_helper.hpp>
#include <actionlib/server/simple_action_server.h>
#include <lesson_actions/CalcPiAction.h>

typedef actionlib::SimpleActionServer<lesson_actions::CalcPiAction> ActionServer;
boost::shared_ptr<ActionServer> as_; 

void executeCB(const lesson_actions::CalcPiGoalConstPtr &goal)
{
  lesson_actions::CalcPiFeedback feedback;
  lesson_actions::CalcPiResult   result;

  double calcPi=0, err=1;
  int iter=0, validDigits=0;

  ROS_INFO("Calculating pi to %i digits", goal->digits);

  // continue processing until converged to specified tolerance
  while (log10(err) >= -goal->digits-1)
  {
    iter++;

    // check for cancel
    if (as_->isPreemptRequested() || !ros::ok())
    {
      ROS_INFO("CalcPi Canceled");
      as_->setPreempted(); // set the action state to preempted
      return;
    }

    // update estimate of Pi
    err = updatePi(calcPi);

    // publish feedback, if new digit or many iterations
    bool newDigit = log10(err) < -(validDigits+2);
    if ( newDigit || iter%(int)1e7==0)
    {
      if (newDigit) validDigits++;

      feedback.iter = iter;
      feedback.pi = toString(calcPi, validDigits);
      as_->publishFeedback(feedback); // publish the feedback

      if (newDigit) ROS_INFO_STREAM("new digit: " << feedback.pi);
    }
  }

  result.pi = feedback.pi;
  ROS_INFO("CalcPi Succeeded");
  as_->setSucceeded(result);  // send result
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "calcPi_ServerNode");

  as_.reset(new ActionServer("calcPiAction", executeCB, false));
  as_->start();
  ROS_INFO("CalcPi server started");

  ros::spin();
  // cleaning up our global now prevents an error on process cleanup
  as_.reset();

  return 0;
}
