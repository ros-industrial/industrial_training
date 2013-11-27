#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#include <ros/ros.h>
#include <actionlib/client/action_client.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <industrial_utils/param_utils.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "jointaction_client");
  ros::NodeHandle nh;

  //get joint names and add to trajectory
  trajectory_msgs::JointTrajectory trajectory;
  if (!industrial_utils::param::getJointNames("controller_joint_names", trajectory.joint_names))
  {
    ROS_WARN("Unable to read 'controller_joint_names' param.  Using standard 6-DOF joint names.");
  }
  ROS_INFO_STREAM("Loaded " << trajectory.joint_names.size() << " joint names from 'controller_joint_names' parameter");


  srand (time(NULL));

  //Complete joint trajectory by oscillating each joint
  double amplitude(1.0);	//max allowable 1/2 angle, in radians
  ros::Duration pt_time_interval(0.1);
  ros::Duration trajectory_time(rand()%60 + 30);
  ROS_INFO_STREAM("Creating trajectory that will last for " << trajectory_time.toSec() << " seconds, with an amplitude of +/- " << amplitude << " radians.");

  trajectory_msgs::JointTrajectoryPoint pt;
  pt.positions = std::vector<double>(trajectory.joint_names.size(), 0.0);
  pt.velocities = std::vector<double>(trajectory.joint_names.size(), 0.0);
  pt.time_from_start = ros::Duration(0.);

  while (pt.time_from_start < trajectory_time) {
	  pt.time_from_start += pt_time_interval;
	  for (unsigned int ii=0; ii<trajectory.joint_names.size(); ii++) {
		  double last_pt = pt.positions[ii];
		  pt.positions[ii] = amplitude * sin(pt.time_from_start.toSec());
		  pt.velocities[ii] = std::fabs(pt.positions[ii] - last_pt) / pt_time_interval.toSec();
	  }
	  trajectory.points.push_back(pt);
  }

  // create the action client
  actionlib::ActionClient<control_msgs::FollowJointTrajectoryAction> ac(nh, "joint_trajectory_action");
  ros::Rate(1.).sleep();    //wait for 5 sec to connect, rather than spin a thread to check connection

  ROS_INFO("Sending goal to action server.");
  // send trajectory to the action server
  control_msgs::FollowJointTrajectoryActionGoal goal;
  goal.goal.trajectory = trajectory;
  ac.sendGoal(goal.goal);

  //exit
  return 0;
}
