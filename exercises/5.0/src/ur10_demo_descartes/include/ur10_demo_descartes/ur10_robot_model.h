/*
 * ur10_robot_model.h
 *
 *  Created on: Apr 15, 2015
 *      Author: ros-devel
 */

#ifndef UR10_DEMO_DESCARTES_UR10_ROBOT_MODEL_H_
#define UR10_DEMO_DESCARTES_UR10_ROBOT_MODEL_H_

#include <descartes_moveit/moveit_state_adapter.h>
#include <ros/ros.h>
#include <urdf/model.h>
#include <eigen_conversions/eigen_kdl.h>
#include <tf_conversions/tf_kdl.h>
#include <ur10_demo_descartes/ur_moveit_plugin.h>
#include <ur10_demo_descartes/ur_kin.h>


namespace ur10_demo_descartes
{

const std::string UR10_BASE_LINK = "base_link";
const std::string UR10_TIP_LINK = "ee_link";

class UR10RobotModel: public descartes_moveit::MoveitStateAdapter, public ur_kinematics::URKinematicsPlugin
{
public:
  UR10RobotModel();
  virtual ~UR10RobotModel();

  virtual bool initialize(const std::string &robot_description, const std::string& group_name,
                            const std::string& world_frame,const std::string& tcp_frame);

  virtual bool getAllIK(const Eigen::Affine3d &pose, std::vector<std::vector<double> > &joint_poses) const;

  descartes_core::Frame world_to_base_;// world to arm base
  descartes_core::Frame tool_to_tip_; // from arm tool to robot tool

};

} /* namespace ur10_demo_descartes */

#endif /* UR10_DEMO_DESCARTES_UR10_ROBOT_MODEL_H_ */
