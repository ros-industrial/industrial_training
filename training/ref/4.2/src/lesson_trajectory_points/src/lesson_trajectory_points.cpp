/*
 * lesson_trajectory_points.cpp
 *
 *  Created on: Apr 21, 2015
 *      Author: Jorge Nicho
 */

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
  descartes_core::TrajectoryPtPtr cart_point_ptr(new descartes_trajectory::CartTrajectoryPt(tool_pose));

  // Getting nominal cartesian pose
  Eigen::Affine3d ik_pose;
  std::vector<double> seed(robot_model_ptr->getDOF(),0);
  seed[0] = 0;
  seed[1] = -M_PI/6;
  seed[2] = M_PI/8;
  seed[3] = -M_PI/6;
  seed[4] = 0;
  seed[5] = M_PI/2;
  if(cart_point_ptr->getNominalCartPose(seed,*robot_model_ptr,ik_pose))
  {
    ROS_INFO_STREAM("Retrieved nominal cartesian pose");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to get nominal cartesian pose");
    return 1;
  }

  // Getting joint pose from cartesian point
  std::vector<double> joint_pose;
  if(cart_point_ptr->getNominalJointPose(seed,*robot_model_ptr,joint_pose))
  {
    ROS_INFO_STREAM("Retrieved joint pose from cartesian point: ["<<joint_pose[0]<<", "
                    <<joint_pose[1]<<", "
                    <<joint_pose[2]<<", "
                    <<joint_pose[3]<<", "
                    <<joint_pose[4]<<", "
                    <<joint_pose[5]<<"] ");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to get joint pose from cartesian point");
    return 1;
  }


  // Create Cartesian point with rotational freedom about the z axis
  descartes_core::TrajectoryPtPtr free_z_rot_pt(
      new descartes_trajectory::AxialSymmetricPt(tool_pose,0.5f,descartes_trajectory::AxialSymmetricPt::Z_AXIS));

  // Getting all the cartesian poses with discretized rotation about z
  EigenSTL::vector_Affine3d poses;
  free_z_rot_pt->getCartesianPoses(*robot_model_ptr,poses);
  if(!poses.empty())
  {
    ROS_INFO_STREAM("Free z Rotation point has "<<poses.size()<<" poses");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to get cartesian poses");
    return 1;
  }


  // Getting all joint solutions
  std::vector< std::vector<double> > solutions;
  free_z_rot_pt->getJointPoses(*robot_model_ptr,solutions);
  if(!solutions.empty())
  {
    ROS_INFO_STREAM("Found "<<solutions.size()<<" joint solutions");
    for(unsigned int i = 0; i < solutions.size();i++)
    {
      const std::vector<double> &sol = solutions[i];
      ROS_INFO_STREAM("IK solution "<<i<<": ["<<sol[0]<<", "
                      <<sol[1]<<", "
                      <<sol[2]<<", "
                      <<sol[3]<<", "
                      <<sol[4]<<", "
                      <<sol[5]<<"] ");
    }
  }
  else
  {
    ROS_ERROR_STREAM("Failed to get joint poses from Axial Symmetric point");
    return 1;
  }

  // Getting closest joint pose
  std::vector<double> closest_joint_pose;
  joint_pose[1] = joint_pose[1] + M_PI/20;
  joint_pose[2] = joint_pose[2] + M_PI/20;
  joint_pose[4] = joint_pose[4] + M_PI/20;
  joint_pose[5] = joint_pose[5] - M_PI/20;

  if(free_z_rot_pt->getClosestJointPose(joint_pose,*robot_model_ptr,closest_joint_pose))
  {
    ROS_INFO_STREAM("Found closest joint pose : ["<<closest_joint_pose[0]<<", "
                    <<closest_joint_pose[1]<<", "
                    <<closest_joint_pose[2]<<", "
                    <<closest_joint_pose[3]<<", "
                    <<closest_joint_pose[4]<<", "
                    <<closest_joint_pose[5]<<"] ");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to find closest joint pose");
    return 1;
  }


  return 0;
}




