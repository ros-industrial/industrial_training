#include <ros/ros.h>
#include <actionlib/client/action_client.h>
#include <industrial_utils/param_utils.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

typedef actionlib::ActionClient<control_msgs::FollowJointTrajectoryAction> ActionClient;

std::vector<std::string> getJointNames()
{
  #define PARAM_NAME "controller_joint_names"
  std::vector<std::string> joint_names;

  //get joint names and add to trajectory
  if (!industrial_utils::param::getJointNames(PARAM_NAME, "", joint_names))
    ROS_WARN("Unable to read %s param.  Using standard 6-DOF joint names.", PARAM_NAME);
  else
    ROS_INFO("Loaded %d joint names from %s parameter", joint_names.size(), PARAM_NAME);

  return joint_names;
}

trajectory_msgs::JointTrajectory calcTrajectory()
{
  trajectory_msgs::JointTrajectory trajectory;

  trajectory.joint_names = getJointNames();
  int nJnts = trajectory.joint_names.size();

  //Complete joint trajectory by oscillating each joint
  double amplitude(1.0);	//max allowable 1/2 angle, in radians
  ros::Duration pt_time_interval(0.1);
  ros::Duration trajectory_time(30);

  trajectory_msgs::JointTrajectoryPoint pt;
  pt.positions = std::vector<double>(nJnts, 0.0);
  pt.velocities = std::vector<double>(nJnts, 0.0);
  pt.time_from_start = ros::Duration(0.);

  while (pt.time_from_start < trajectory_time) {
	  pt.time_from_start += pt_time_interval;
	  for (unsigned int ii=0; ii<nJnts; ii++) {
		  double last_pt = pt.positions[ii];
		  pt.positions[ii] = amplitude * sin(pt.time_from_start.toSec());
		  pt.velocities[ii] = std::fabs(pt.positions[ii] - last_pt) / pt_time_interval.toSec();
	  }
	  trajectory.points.push_back(pt);
  }

  return trajectory;
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "jointaction_client");
  ros::NodeHandle nh;

  // create the action client
  ActionClient ac(nh, "joint_trajectory_action");
  ros::Rate(1.).sleep();    //wait for 1 sec to connect

  // send trajectory to the action server
  ROS_INFO("Sending goal to action server.");
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = calcTrajectory();
  ac.sendGoal(goal);

  //don't wait for action to complete
  return 0;
}
