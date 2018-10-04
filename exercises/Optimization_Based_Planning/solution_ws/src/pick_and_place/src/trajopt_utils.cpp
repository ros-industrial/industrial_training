#include <test_bed_core/trajopt_utils.h>
#include <trajopt/problem_description.hpp>

#include <ros/ros.h>

#include <trajectory_msgs/JointTrajectory.h>
//#include <trajopt/typedefs.hpp>

// using namespace trajopt;
// using namespace Eigen;


trajectory_msgs::JointTrajectory trajArrayToJointTrajectoryMsg(std::vector<std::string> joint_names,
                                                               tesseract::TrajArray traj_array, ros::Duration time_increment)
{
  trajectory_msgs::JointTrajectory traj_msg;
  traj_msg.header.stamp = ros::Time::now();
  traj_msg.joint_names = joint_names;

  ros::Duration time_from_start = time_increment;
  for (int ind = 0; ind < traj_array.rows(); ind++)
  {
    trajectory_msgs::JointTrajectoryPoint traj_point;
    auto mat = traj_array.row(ind);
    std::vector<double> vec(mat.data(), mat.data() + mat.rows() * mat.cols());
    traj_point.positions = vec;
    traj_point.time_from_start = time_from_start;
    time_from_start += time_increment;

    traj_msg.points.push_back(traj_point);

  }
  return traj_msg;
}
