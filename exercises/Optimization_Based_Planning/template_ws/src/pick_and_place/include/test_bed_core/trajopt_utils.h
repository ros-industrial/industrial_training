#pragma once

#include <trajectory_msgs/JointTrajectory.h>
//#include <trajopt/typedefs.hpp>
#include <tesseract_planning/basic_planner_types.h>

trajectory_msgs::JointTrajectory trajArrayToJointTrajectoryMsg(std::vector<std::string> joint_names, tesseract::TrajArray traj_array, ros::Duration time_increment);

