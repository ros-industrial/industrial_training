/*
 * demo_application.h
 *
 *  Created on: Apr 9, 2015
 *      Author: ros-devel
 */

#ifndef DEMO_DESCARTES_PLAN_AND_RUN_INCLUDE_PLAN_AND_RUN_DEMO_APPLICATION_H_
#define DEMO_DESCARTES_PLAN_AND_RUN_INCLUDE_PLAN_AND_RUN_DEMO_APPLICATION_H_

#include <ros/ros.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit/move_group_interface/move_group.h>
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>
#include <descartes_planner/dense_planner.h>
#include <descartes_planner/sparse_planner.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen_conversions/eigen_msg.h>
#include <ur5_demo_descartes/ur5_robot_model.h>

namespace plan_and_run
{

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";
const std::string EXECUTE_TRAJECTORY_SERVICE = "execute_kinematic_path";
const std::string VISUALIZE_TRAJECTORY_TOPIC = "visualize_trajectory_curve";
const double SERVICE_TIMEOUT = 5.0f; // seconds
const double ORIENTATION_INCREMENT = 0.5f;
const double EPSILON = 0.0001f;
const double AXIS_LINE_LENGHT = 0.01;
const double AXIS_LINE_WIDTH = 0.002;
const std::string PLANNER_ID = "RRTConnectkConfigDefault";
const std::string HOME_POSITION_NAME = "home";

typedef std::vector<descartes_core::TrajectoryPtPtr> DescartesTrajectory;

struct DemoConfiguration
{
  std::string group_name;
  std::string tip_link;
  std::string base_link;
  std::string world_frame;
  std::vector<std::string> joint_names;

  // trajectory generation
  double time_delay;
  double foci_distance;
  double radius;
  int num_points;
  int num_lemniscates;
  std::vector<double> center;
  std::vector<double> seed_pose;
};

class DemoApplication
{
public:
  DemoApplication();
  virtual ~DemoApplication();

  void loadParameters();
  void initMoveitClient();
  void initDescartes();
  void moveHome();
  void generateTrajectory(DescartesTrajectory& traj);
  void planPath(DescartesTrajectory& input_traj,DescartesTrajectory& output_path);
  void runPath(const DescartesTrajectory& path);


  // utility methods

  static bool createLemniscateCurve(double foci_distance, double sphere_radius,
                                    int num_points, int num_lemniscates,
                                    const Eigen::Vector3d& sphere_center,
                                    EigenSTL::vector_Affine3d& poses);

protected:

  void fromDescartesToMoveitTrajectory(const DescartesTrajectory& in_traj,
                                              trajectory_msgs::JointTrajectory& out_traj);
  void publishPosesMarkers(const EigenSTL::vector_Affine3d& poses);


protected:

  ros::NodeHandle nh_;
  DemoConfiguration config_;
  ros::Publisher marker_publisher_;
  ros::ServiceClient moveit_run_path_client_;

  descartes_core::RobotModelPtr robot_model_ptr_;
  descartes_planner::SparsePlanner planner_;

};

} /* namespace plan_and_run */

#endif /* DEMO_DESCARTES_PLAN_AND_RUN_INCLUDE_PLAN_AND_RUN_DEMO_APPLICATION_H_ */
