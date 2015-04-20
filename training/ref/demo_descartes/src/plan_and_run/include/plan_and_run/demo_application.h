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
const double AXIS_LINE_WIDTH = 0.001;
const std::string PLANNER_ID = "RRTConnectkConfigDefault";
const std::string HOME_POSITION_NAME = "home";

typedef std::vector<descartes_core::TrajectoryPtPtr> DescartesTrajectory;

/*  =============================== Application Data Structure ===============================
 *
 * Holds the data used at various points in the application.  This structure is populated
 * from data found in the ros parameter server at runtime.
 *
 */
struct DemoConfiguration
{
  std::string group_name;                 /* Name of the manipulation group containing the relevant links in the robot */
  std::string tip_link;                   /* Usually the last link in the kinematic chain of the robot */
  std::string base_link;                  /* The name of the base link of the robot */
  std::string world_frame;                /* The name of the world link in the URDF file */
  std::vector<std::string> joint_names;   /* A list with the names of the mobile joints in the robot */


  /* Trajectory Generation Members:
   *  Used to control the attributes (points, shape, size, etc) of the robot trajectory.
   *  */
  double time_delay;              /* Time step between consecutive points in the robot path */
  double foci_distance;           /* Controls the size of the curve */
  double radius;                  /* Controls the radius of the sphere on which the curve is projected */
  int num_points;                 /* Number of points per curve */
  int num_lemniscates;            /* Number of curves*/
  std::vector<double> center;     /* Location of the center of all the lemniscate curves */
  std::vector<double> seed_pose;  /* Joint values close to the desired start of the robot path */

  /*
   * Visualization Members
   * Used to control the attributes of the visualization artifacts
   */
  double min_point_distance;      /* Minimum distance between consecutive trajectory points. */
};


/*  =============================== Application Class ===============================
 *
 * Provides a group of functions for planning and executing a robot path using Moveit and
 * the Descartes Planning Library
 *
 */
class DemoApplication
{
public:
  /*  Constructor
   *    Creates an instance of the application class
   */
  DemoApplication();
  virtual ~DemoApplication();

  /* Main Application Functions
   *  Functions that allow carrying out the various steps needed to run a
   *  plan an run application.  All these functions will be invoked from within
   *  the main routine.
   */

  void loadParameters();
  void initRos();
  void initDescartes();
  void moveHome();
  void generateTrajectory(DescartesTrajectory& traj);
  void planPath(DescartesTrajectory& input_traj,DescartesTrajectory& output_path);
  void runPath(const DescartesTrajectory& path);

protected:

  /* Support methods
   *  Called from within the main application functions in order to perform convenient tasks.
   */

  static bool createLemniscateCurve(double foci_distance, double sphere_radius,
                                    int num_points, int num_lemniscates,
                                    const Eigen::Vector3d& sphere_center,
                                    EigenSTL::vector_Affine3d& poses);

  void fromDescartesToMoveitTrajectory(const DescartesTrajectory& in_traj,
                                              trajectory_msgs::JointTrajectory& out_traj);

  void publishPosesMarkers(const EigenSTL::vector_Affine3d& poses);


protected:

  /* Application Data
   *  Holds the data used by the various functions in the application.
   */
  DemoConfiguration config_;



  /* Application ROS Constructs
   *  Components needed to successfully run a ros-node and perform other important
   *  ros-related tasks
   */
  ros::NodeHandle nh_;                        /* Object used for creating and managing ros application resources*/
  ros::Publisher marker_publisher_;           /* Publishes visualization message to Rviz */
  ros::ServiceClient moveit_run_path_client_; /* Sends a robot trajectory to moveit for execution */



  /* Application Descartes Constructs
   *  Components accessing the path planning capabilities in the Descartes library
   */
  descartes_core::RobotModelPtr robot_model_ptr_; /* Performs tasks specific to the Robot
                                                     such IK, FK and collision detection*/
  descartes_planner::SparsePlanner planner_;      /* Plans a smooth robot path given a trajectory of points */

};

} /* namespace plan_and_run */

#endif /* DEMO_DESCARTES_PLAN_AND_RUN_INCLUDE_PLAN_AND_RUN_DEMO_APPLICATION_H_ */
