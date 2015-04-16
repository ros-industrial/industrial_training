/*
 * demo_application.cpp
 *
 *  Created on: Apr 9, 2015
 *      Author: ros-devel
 */

#include <plan_and_run/demo_application.h>

namespace plan_and_run
{

DemoApplication::DemoApplication()
{
  // TODO Auto-generated constructor stub

}

DemoApplication::~DemoApplication()
{
  // TODO Auto-generated destructor stub
}

void DemoApplication::loadParameters()
{
  ros::NodeHandle ph("~");
  ros::NodeHandle nh;

  if(ph.getParam("group_name",config_.group_name) &&
      ph.getParam("tip_link",config_.tip_link) &&
      ph.getParam("base_link",config_.base_link) &&
      ph.getParam("world_frame",config_.world_frame) &&
      ph.getParam("trajectory/time_delay",config_.time_delay) &&
      ph.getParam("trajectory/foci_distance",config_.foci_distance) &&
      ph.getParam("trajectory/radius",config_.radius) &&
      ph.getParam("trajectory/num_points",config_.num_points) &&
      ph.getParam("trajectory/num_lemniscates",config_.num_lemniscates) &&
      ph.getParam("trajectory/center",config_.center) &&
      ph.getParam("trajectory/seed_pose",config_.seed_pose) &&
      nh.getParam("controller_joint_names",config_.joint_names) )
  {
    ROS_INFO_STREAM("Loaded application parameters");

  }
  else
  {
    ROS_ERROR_STREAM("Failed to load application parameters");
    exit(-1);
  }

}

void DemoApplication::initDescartes()
{
  // Robot Model initialization
  robot_model_ptr_.reset(new ur5_demo_descartes::UR5RobotModel());
  if(robot_model_ptr_->initialize(ROBOT_DESCRIPTION_PARAM,
                                  config_.group_name,
                                  config_.world_frame,
                                  config_.tip_link))
  {
    ROS_INFO_STREAM("Descartes Robot Model initialized");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to initialize Robot Model");
    exit(-1);
  }

  // Planner initialization
  if(planner_.initialize(robot_model_ptr_))
  {
    ROS_INFO_STREAM("Descartes Dense Planner initialized");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to initialize Dense Planner");
    exit(-1);
  }

}

void DemoApplication::initMoveitClient()
{
  // creating publisher for trajectory visualization
  marker_publisher_  = nh_.advertise<visualization_msgs::MarkerArray>(VISUALIZE_TRAJECTORY_TOPIC,1,true);

  // creating client for requesting execution of the robot path
  moveit_run_path_client_ = nh_.serviceClient<moveit_msgs::ExecuteKnownTrajectory>(EXECUTE_TRAJECTORY_SERVICE,true);
  if(moveit_run_path_client_.waitForExistence(ros::Duration(SERVICE_TIMEOUT)))
  {
    ROS_INFO_STREAM("Connected to '"<<moveit_run_path_client_.getService()<<"' service");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to connect to '"<< moveit_run_path_client_.getService()<<"' service");
    exit(-1);
  }


}

void DemoApplication::moveHome()
{
  move_group_interface::MoveGroup move_group(config_.group_name);
  move_group.setPlannerId(PLANNER_ID);

  if(!move_group.setNamedTarget(HOME_POSITION_NAME))
  {
    ROS_ERROR_STREAM("Failed to set home '"<<HOME_POSITION_NAME<<"' position");
    exit(-1);
  }

  // moving home
  moveit_msgs::MoveItErrorCodes result =  move_group.move();
  if(result.val != result.SUCCESS)
  {
    ROS_ERROR_STREAM("Failed to move to "<<HOME_POSITION_NAME<<" position");
    exit(-1);
  }

  ROS_INFO_STREAM("Robot reached home position");

}


void DemoApplication::generateTrajectory(DescartesTrajectory& traj)
{

  using namespace descartes_core;
  using namespace descartes_trajectory;

  // generating lemniscate curve
  EigenSTL::vector_Affine3d poses;
  Eigen::Vector3d center(config_.center[0],config_.center[1],config_.center[2]);

  if(createLemniscateCurve(config_.foci_distance,config_.radius,config_.num_points,
                        config_.num_lemniscates,center,poses))
  {
    ROS_INFO_STREAM("Trajectory with "<<poses.size()<<" points was generated");
  }
  else
  {
    ROS_ERROR_STREAM("Trajectory generation failed");
    exit(-1);
  }

  // publishing trajectory poses for visualization
  publishPosesMarkers(poses);

  // creating descartes trajectory points
  traj.clear();
  traj.reserve(poses.size());
  for(unsigned int i = 0; i < poses.size(); i++)
  {
    const Eigen::Affine3d& pose = poses[i];

    // creating cartesian point with rotational freedom about the tool's approach axis
    descartes_core::TrajectoryPtPtr pt = descartes_core::TrajectoryPtPtr(
        new descartes_trajectory::AxialSymmetricPt(pose,ORIENTATION_INCREMENT,
                                                   descartes_trajectory::AxialSymmetricPt::FreeAxis::Z_AXIS) );

    traj.push_back(pt);
  }

}

void DemoApplication::publishPosesMarkers(const EigenSTL::vector_Affine3d& poses)
{
  // creating rviz markers
  visualization_msgs::Marker z_axes, y_axes, x_axes, line;
  visualization_msgs::MarkerArray markers_msg;

  z_axes.type = y_axes.type = x_axes.type = visualization_msgs::Marker::LINE_LIST;
  z_axes.ns = y_axes.ns = x_axes.ns = "axes";
  z_axes.action = y_axes.action = x_axes.action = visualization_msgs::Marker::ADD;
  z_axes.lifetime = y_axes.lifetime = x_axes.lifetime = ros::Duration(0);
  z_axes.header.frame_id = y_axes.header.frame_id = x_axes.header.frame_id = config_.world_frame;
  z_axes.scale.x = y_axes.scale.x = x_axes.scale.x = AXIS_LINE_WIDTH;

  // z properties
  z_axes.id = 0;
  z_axes.color.r = 0;
  z_axes.color.g = 0;
  z_axes.color.b = 1;
  z_axes.color.a = 1;

  // y properties
  y_axes.id = 1;
  y_axes.color.r = 0;
  y_axes.color.g = 1;
  y_axes.color.b = 0;
  y_axes.color.a = 1;

  // x properties
  x_axes.id = 2;
  x_axes.color.r = 1;
  x_axes.color.g = 0;
  x_axes.color.b = 0;
  x_axes.color.a = 1;

  // line properties
  line.type = visualization_msgs::Marker::LINE_STRIP;
  line.ns = "line";
  line.action = visualization_msgs::Marker::ADD;
  line.lifetime = ros::Duration(0);
  line.header.frame_id = config_.world_frame;
  line.scale.x = AXIS_LINE_WIDTH;
  line.id = 0;
  line.color.r = 1;
  line.color.g = 1;
  line.color.b = 0;
  line.color.a = 1;

  // creating axes markers
  z_axes.points.reserve(2*poses.size());
  y_axes.points.reserve(2*poses.size());
  x_axes.points.reserve(2*poses.size());
  line.points.reserve(poses.size());
  geometry_msgs::Point p_start,p_end;
  for(unsigned int i = 0; i < poses.size(); i++)
  {
    const Eigen::Affine3d& pose = poses[i];

    Eigen::Affine3d moved_along_x = pose * Eigen::Translation3d(AXIS_LINE_LENGHT,0,0);
    tf::pointEigenToMsg(pose.translation(),p_start);
    tf::pointEigenToMsg(moved_along_x.translation(),p_end);
    x_axes.points.push_back(p_start);
    x_axes.points.push_back(p_end);

    Eigen::Affine3d moved_along_y = pose * Eigen::Translation3d(0,AXIS_LINE_LENGHT,0);
    tf::pointEigenToMsg(moved_along_y.translation(),p_end);
    y_axes.points.push_back(p_start);
    y_axes.points.push_back(p_end);

    Eigen::Affine3d moved_along_z = pose * Eigen::Translation3d(0,0,AXIS_LINE_LENGHT);
    tf::pointEigenToMsg(moved_along_z.translation(),p_end);
    z_axes.points.push_back(p_start);
    z_axes.points.push_back(p_end);

    line.points.push_back(p_start);
  }

  markers_msg.markers.push_back(x_axes);
  markers_msg.markers.push_back(y_axes);
  markers_msg.markers.push_back(z_axes);
  markers_msg.markers.push_back(line);

  marker_publisher_.publish(markers_msg);

}

bool DemoApplication::createLemniscateCurve(double foci_distance, double sphere_radius,
                                  int num_points, int num_lemniscates,const Eigen::Vector3d& sphere_center,
                                  EigenSTL::vector_Affine3d& poses)
{
  double a = foci_distance;
  double ro = sphere_radius;
  int npoints = num_points;
  int nlemns = num_lemniscates;
  Eigen::Vector3d offset(sphere_center[0],sphere_center[1],sphere_center[2]);
  Eigen::Vector3d unit_z,unit_y,unit_x;

  // checking parameters
  if(a <= 0 || ro <= 0 || npoints < 10 || nlemns < 1)
  {
    ROS_ERROR_STREAM("Invalid parameters for lemniscate curve were found");
    return false;
  }

  // generating polar angle values
  std::vector<double> theta(npoints);

  // interval 1 <-pi/4 , pi/4 >
  double d_theta = 2*M_PI_2/(npoints - 1);
  for(unsigned int i = 0; i < npoints/2;i++)
  {
    theta[i] = -M_PI_4  + i * d_theta;
  }
  theta[0] = theta[0] + EPSILON;
  theta[npoints/2 - 1] = theta[npoints/2 - 1] - EPSILON;

  // interval 2 < 3*pi/4 , 5 * pi/4 >
  for(unsigned int i = 0; i < npoints/2;i++)
  {
    theta[npoints/2 + i] = 3*M_PI_4  + i * d_theta;
  }
  theta[npoints/2] = theta[npoints/2] + EPSILON;
  theta[npoints - 1] = theta[npoints - 1] - EPSILON;

  // generating omega angle (lemniscate angle offset)
  std::vector<double> omega(nlemns);
  double d_omega = M_PI/(nlemns);
  for(unsigned int i = 0; i < nlemns;i++)
  {
     omega[i] = i*d_omega;
  }

  Eigen::Affine3d pose;
  double x,y,z,r,phi;

  poses.clear();
  poses.reserve(nlemns*npoints);
  for(unsigned int j = 0; j < nlemns;j++)
  {
    for(unsigned int i = 0 ; i < npoints;i++)
    {
      r = std::sqrt( std::pow(a,2) * std::cos(2*theta[i]) );
      phi = r < ro ? std::asin(r/ro):  (M_PI - std::asin((2*ro - r)/ro) );

      x = ro * std::cos(theta[i] + omega[j]) * std::sin(phi);
      y = ro * std::sin(theta[i] + omega[j]) * std::sin(phi);
      z = ro * std::cos(phi);

      // determining orientation
      unit_z <<-x, -y , -z;
      unit_z.normalize();

      unit_x = (Eigen::Vector3d(0,1,0).cross( unit_z)).normalized();
      unit_y = (unit_z .cross(unit_x)).normalized();

      Eigen::Matrix3d rot;
      rot << unit_x(0),unit_y(0),unit_z(0)
         ,unit_x(1),unit_y(1),unit_z(1)
         ,unit_x(2),unit_y(2),unit_z(2);

      pose = Eigen::Translation3d(offset(0) + x,
                                  offset(1) + y,
                                  offset(2) + z) * rot;

      poses.push_back(pose);
    }
  }

  return true;
}

void DemoApplication::planPath(DescartesTrajectory& input_traj,DescartesTrajectory& output_path)
{
  // modifying start and end points
  std::vector<double> start_pose, end_pose;
  if(input_traj.front()->getClosestJointPose(config_.seed_pose,*robot_model_ptr_,start_pose) &&
      input_traj.back()->getClosestJointPose(config_.seed_pose,*robot_model_ptr_,end_pose))
  {
    ROS_INFO_STREAM("Setting trajectory start and end to JointTrajectoryPts");
    descartes_core::TrajectoryPtPtr start_joint_point = descartes_core::TrajectoryPtPtr(
        new descartes_trajectory::JointTrajectoryPt(start_pose));
    descartes_core::TrajectoryPtPtr end_joint_point = descartes_core::TrajectoryPtPtr(
        new descartes_trajectory::JointTrajectoryPt(end_pose));

    input_traj[0] = start_joint_point;
    input_traj[input_traj.size() - 1 ] = end_joint_point;
  }
  else
  {
    ROS_ERROR_STREAM("Failed to find closest joint pose to seed pose at the start or end of trajectory");
    exit(-1);
  }

  // planning robot path
  if (planner_.planPath(input_traj))
  {
    ROS_INFO_STREAM("Valid path was found");
  }
  else
  {
    ROS_ERROR_STREAM("Could not solve for a valid path");
    exit(-1);
  }

  // retrieving robot path
  if(!planner_.getPath(output_path) || output_path.empty())
  {
    ROS_ERROR_STREAM("Failed to retrieve robot path");
  }
}

void DemoApplication::runPath(const DescartesTrajectory& path)
{

  // creating move group to move the arm in free space
  move_group_interface::MoveGroup move_group(config_.group_name);
  move_group.setPlannerId(PLANNER_ID);

  // creating goal joint pose to start of the path
  std::vector<double> seed_pose(robot_model_ptr_->getDOF()),start_pose;
  path[0]->getNominalJointPose(seed_pose,*robot_model_ptr_,start_pose);

  // moving arm to joint goal
  move_group.setJointValueTarget(start_pose);
  move_group.setPlanningTime(10.0f);
  moveit_msgs::MoveItErrorCodes result = move_group.move();
  if(result.val != result.SUCCESS)
  {
    ROS_ERROR_STREAM("Move to start joint pose failed");
    exit(-1);
  }

  // sending path to robot
  moveit_msgs::RobotTrajectory moveit_traj;
  fromDescartesToMoveitTrajectory(path,moveit_traj.joint_trajectory);
  moveit_msgs::ExecuteKnownTrajectory srv;
  srv.request.trajectory = moveit_traj;
  srv.request.wait_for_execution = true;

  ROS_INFO_STREAM("Robot path sent for execution");
  if(moveit_run_path_client_.call(srv))
  {
    ROS_INFO_STREAM("Robot path execution completed");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to run robot path with error "<<srv.response.error_code.val);
    exit(-1);
  }

}

void DemoApplication::fromDescartesToMoveitTrajectory(const DescartesTrajectory& in_traj,
                                                      trajectory_msgs::JointTrajectory& out_traj)
{
  // Fill out information about our trajectory
  out_traj.header.stamp = ros::Time::now();
  out_traj.header.frame_id = config_.world_frame;
  out_traj.joint_names = config_.joint_names;

  // For keeping track of time-so-far in the trajectory
  double time_offset = 0.0;

  // Loop through the trajectory
  for (unsigned int i = 0; i < in_traj.size(); i++)
  {
    // Find nominal joint solution at this point
    std::vector<double> joints;

    // getting joint position at current point
    const descartes_core::TrajectoryPtPtr& joint_point = in_traj[i];
    joint_point->getNominalJointPose(std::vector<double>(), *robot_model_ptr_, joints);

    // Fill out a ROS trajectory point
    trajectory_msgs::JointTrajectoryPoint pt;
    pt.positions = joints;
    // velocity, acceleration, and effort are given dummy values
    // we'll let the controller figure them out
    pt.velocities.resize(joints.size(), 0.0);
    pt.accelerations.resize(joints.size(), 0.0);
    pt.effort.resize(joints.size(), 0.0);
    // set the time into the trajectory
    pt.time_from_start = ros::Duration(time_offset);
    // increment time
    time_offset += config_.time_delay;

    out_traj.points.push_back(pt);
  }

}

} /* namespace plan_and_run */
