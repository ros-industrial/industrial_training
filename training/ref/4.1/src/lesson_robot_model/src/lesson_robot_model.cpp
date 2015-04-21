#include <ros/ros.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit/move_group_interface/move_group.h>
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>
#include <descartes_moveit/moveit_state_adapter.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen_conversions/eigen_msg.h>


int main(int argc, char** argv)
{
  ros::init(argc,argv,"lesson_descartes_basic");
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


  // Perform FK with a set of joint values
  std::vector<double> joints(robot_model_ptr->getDOF(),0);
  joints[0] = 0;
  joints[1] = -M_PI/6;
  joints[2] = M_PI/8;
  joints[3] = -M_PI/6;
  joints[4] = 0;
  joints[5] = M_PI/2;

  Eigen::Affine3d tool_pose;
  if(robot_model_ptr->getFK(joints,tool_pose))
  {
    ROS_INFO_STREAM("FK succeeded");
  }
  else
  {
    ROS_ERROR_STREAM("FK failed");
    return 1;
  }

  // Perform IK to get a single joint solution
  std::vector<double> solution;
  if(robot_model_ptr->getIK(tool_pose,joints,solution))
  {
    ROS_INFO_STREAM("Found IK solution: ["<<solution[0]<<", "
                    <<solution[1]<<", "
                    <<solution[2]<<", "
                    <<solution[3]<<", "
                    <<solution[4]<<", "
                    <<solution[5]<<"] ");
  }
  else
  {
    ROS_ERROR_STREAM("IK failed");
    return 1;
  }

  // Perform IK to get multiple valid solutions
  std::vector< std::vector<double> > solutions;
  if(robot_model_ptr->getAllIK(tool_pose,solutions))
  {
    ROS_INFO_STREAM("Found "<<solutions.size()<< " IK solutions");

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
    ROS_ERROR_STREAM("IK failed");
    return 1;
  }

  return 0;
}
