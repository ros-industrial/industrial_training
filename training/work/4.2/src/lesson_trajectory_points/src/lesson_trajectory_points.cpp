/*
 * lesson_trajectory_points.cpp
 *
 *  Created on: Apr 21, 2015
 *      Author: Jorge Nicho
 */

#ifdef __i386__
#pragma message("i386 Architecture detected, disabling EIGEN VECTORIZATION")
#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#else
#pragma message("64bit Architecture detected, enabling EIGEN VECTORIZATION")
#endif

#include <ros/ros.h>
#include <descartes_moveit/moveit_state_adapter.h>
#include <descartes_trajectory/cart_trajectory_pt.h>
#include <descartes_trajectory/joint_trajectory_pt.h>
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <eigen_conversions/eigen_msg.h>


int main(int argc, char** argv)
{
  ros::init(argc,argv,"lesson_trajectory_points");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Create a Descartes robot model
  descartes_core::RobotModelPtr robot_model_ptr(new descartes_moveit::MoveitStateAdapter());
  if(robot_model_ptr->initialize("robot_description","manipulator","world","tool"))
  {
    ROS_INFO_STREAM("Robot Model initialized");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to initialized Robot Model");
    return 1;
  }

  // Create Descartes cartesian point
  Eigen::Affine3d tool_pose = descartes_core::utils::toFrame(0.4f,0.2f,0.1f,0,M_PI/2,0);


  // Getting nominal cartesian pose
  Eigen::Affine3d ik_pose;
  std::vector<double> seed(robot_model_ptr->getDOF(),0);
  seed[0] = 0;
  seed[1] = -M_PI/6;
  seed[2] = M_PI/8;
  seed[3] = -M_PI/6;
  seed[4] = 0;
  seed[5] = M_PI/2;

  // Getting joint pose from cartesian point
  std::vector<double> joint_pose;


  // Create Cartesian point with rotational freedom about the z axis


  // Getting all the cartesian poses with discretized rotation about z
  EigenSTL::vector_Affine3d poses;


  // Getting all joint solutions
  std::vector< std::vector<double> > solutions;


  // Getting closest joint pose
  std::vector<double> closest_joint_pose;
  joint_pose[1] = joint_pose[1] + M_PI/20;
  joint_pose[2] = joint_pose[2] + M_PI/20;
  joint_pose[4] = joint_pose[4] + M_PI/20;
  joint_pose[5] = joint_pose[5] - M_PI/20;


  return 0;
}




