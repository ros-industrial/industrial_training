/*
 * lesson_trajectory_points.cpp
 *
 *  Created on: Apr 21, 2015
 *      Author: Jorge Nicho
 */

#include <ros/ros.h>
#include <descartes_moveit/moveit_state_adapter.h>
#include <eigen_conversions/eigen_msg.h>


int main(int argc, char** argv)
{
  ros::init(argc,argv,"lesson_trajectory_points");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Create a Descartes robot model
  descartes_core::RobotModelPtr robot_model_ptr(new descartes_moveit::MoveitStateAdapter());



  // Perform FK with a set of joint values
  std::vector<double> joints(robot_model_ptr->getDOF(),0);
  joints[0] = 0;
  joints[1] = -M_PI/6;
  joints[2] = M_PI/8;
  joints[3] = -M_PI/6;
  joints[4] = 0;
  joints[5] = M_PI/2;

  Eigen::Affine3d tool_pose;


  // Perform IK to get a single joint solution
  std::vector<double> solution;


  // Perform IK to get multiple valid solutions
  std::vector< std::vector<double> > solutions;


  return 0;
}
