/*
 * lesson_path_planning.cpp
 *
 *  Created on: Apr 22, 2015
 *      Author: Jorge Nicho
 */

#include <ros/ros.h>
#include <descartes_moveit/moveit_state_adapter.h>
#include <descartes_trajectory/cart_trajectory_pt.h>
#include <descartes_trajectory/joint_trajectory_pt.h>
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_planner/dense_planner.h>
#include <descartes_planner/sparse_planner.h>

#include <eigen_conversions/eigen_msg.h>


int main(int argc, char** argv)
{
  ros::init(argc,argv,"lesson_path_planning");
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

  // Creating trajectory
  const double d_pos = 0.02f;
  const double d_rz = M_PI/20;
  const int num_points = 20;
  std::vector<descartes_core::TrajectoryPtPtr> traj;
  traj.reserve(num_points);
  for(unsigned int i = 0; i < num_points; i++)
  {
    Eigen::Affine3d tool_pose = descartes_core::utils::toFrame(0.2f + d_pos*i,  //x
                                                               -0.2f + d_pos*i, //y
                                                               0.1f,            //z
                                                               0,               //rx
                                                               M_PI/2,          //ry
                                                               d_rz * i);       // rz

    descartes_core::TrajectoryPtPtr cart_point(
        new descartes_trajectory::AxialSymmetricPt(tool_pose,1.0f,descartes_trajectory::AxialSymmetricPt::Z_AXIS));
    traj.push_back(cart_point);
  }

  // Creating dense planner
  descartes_planner::DensePlanner planner;
  if(planner.initialize(robot_model_ptr))
  {
    ROS_INFO_STREAM("Planner initialized");
  }
  else
  {
    ROS_ERROR_STREAM("Planner initialization failed");
    return 1;
  }

  // Planning robot path
  std::vector<descartes_core::TrajectoryPtPtr> path;
  if(planner.planPath(traj))
  {
    if(planner.getPath(path))
    {
      ROS_INFO_STREAM("Path planning completed with "<<path.size()<<" points");
    }
    else
    {
      ROS_ERROR_STREAM("Path planning failed");
      return 1;
    }

  }
  else
  {
    ROS_ERROR_STREAM("Path planning failed");
    return 1;
  }

  // Print all joint solutions
  std::vector<double> seed(robot_model_ptr->getDOF());
  for(unsigned int i = 0; i < path.size(); i++)
  {
    std::vector<double> joints;
    descartes_core::TrajectoryPtPtr joint_pt = path[i];
    joint_pt->getNominalJointPose(seed,*robot_model_ptr,joints);

    ROS_INFO_STREAM("Joint point "<<i<<": ["
                    <<joints[0]<<", "
                    <<joints[1]<<", "
                    <<joints[2]<<", "
                    <<joints[3]<<", "
                    <<joints[4]<<", "
                    <<joints[5]<<"] ");
  }

  // Planning with the sparse planner
  descartes_planner::SparsePlanner sparse_planner;
  if(sparse_planner.initialize(robot_model_ptr))
  {
    ROS_INFO_STREAM("Sparse Planner initialized");
  }
  else
  {
    ROS_ERROR_STREAM("Sparse Planner initialization failed");
    return 1;
  }

  path.clear();
  if(sparse_planner.planPath(traj))
  {
    if(sparse_planner.getPath(path))
    {
      ROS_INFO_STREAM("Sparse planner completed with "<<path.size()<<" points");
    }
    else
    {
      ROS_ERROR_STREAM("Path planning failed");
      return 1;
    }

  }
  else
  {
    ROS_ERROR_STREAM("Path planning failed");
    return 1;
  }



  return 0;

}
