#include <ros/ros.h>
#include <ros/package.h>
#include "myworkcell_core/PlanCartesianPath.h"

#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <ur5_demo_descartes/ur5_robot_model.h>
#include <descartes_planner/dense_planner.h>
#include <descartes_planner/sparse_planner.h>
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/joint_trajectory_pt.h>
#include <descartes_utilities/ros_conversions.h>
#include <eigen_conversions/eigen_msg.h>
#include <fstream>
#include <string>

std::vector<double> getCurrentJointState(const std::string& topic)
{
  sensor_msgs::JointStateConstPtr state = ros::topic::waitForMessage<sensor_msgs::JointState>(topic, ros::Duration(0.0));
  if (!state) throw std::runtime_error("Joint state message capture failed");
  return state->position;
}

EigenSTL::vector_Isometry3d makeLine(const Eigen::Vector3d& start, const Eigen::Vector3d& stop, double ds)
{
  EigenSTL::vector_Isometry3d line;

  const Eigen::Vector3d travel = stop - start;
  const int steps = std::floor(travel.norm() / ds);

  // Linear interpolation
  for (int i = 0; i < steps; ++i)
  {
    double ratio = static_cast<float>(i) / steps;
    Eigen::Vector3d position = start + ratio * travel;
    Eigen::Isometry3d tr;
    tr = Eigen::Translation3d(position);
    line.push_back( tr );
  }

  return line;
}

class CartesianPlanner
{
public:
  CartesianPlanner(ros::NodeHandle& nh)
  {
    // first init descartes
    if (!initDescartes())
      throw std::runtime_error("There was an issue initializing Descartes");

    // init services
    server_ = nh.advertiseService("adv_plan_path", &CartesianPlanner::planPath, this);

    vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>("puzzle_path", 0);
  }

  bool initDescartes()
  {
    // Create a robot model
    model_ = boost::make_shared<ur5_demo_descartes::UR5RobotModel>();

    // Define the relevant "frames"
    const std::string robot_description = "robot_description";
    const std::string group_name = "puzzle";
    const std::string world_frame = "world"; // Frame in which tool poses are expressed
    const std::string tcp_frame = "part";

    // Using the desired frames, let's initialize Descartes
    if (!model_->initialize(robot_description, group_name, world_frame, tcp_frame))
    {
      ROS_WARN("Descartes RobotModel failed to initialize");
      return false;
    }
    model_->setCheckCollisions(true);

    if (!planner_.initialize(model_))
    {
      ROS_WARN("Descartes Planner failed to initialize");
      return false;
    }

    return true;
  }

  bool planPath(myworkcell_core::PlanCartesianPathRequest& req,
                myworkcell_core::PlanCartesianPathResponse& res)
  {
    ROS_INFO("Recieved cartesian planning request");

    // Step 1: Generate path poses
    EigenSTL::vector_Isometry3d tool_poses = makePuzzleToolPoses();//makeToolPoses();
    visualizePuzzlePath(tool_poses);

    // Step 2: Translate that path by the input reference pose and convert to "Descartes points"
    std::vector<descartes_core::TrajectoryPtPtr> path = makeDescartesTrajectory(tool_poses);

    // Step 3: Tell Descartes to start at the "current" robot position
    std::vector<double> start_joints = getCurrentJointState("joint_states");
    descartes_core::TrajectoryPtPtr pt (new descartes_trajectory::JointTrajectoryPt(start_joints));
    path.front() = pt;

    // Step 4: Plan with descartes
    if (!planner_.planPath(path))
    {
      return false;
    }

    std::vector<descartes_core::TrajectoryPtPtr> result;
    if (!planner_.getPath(result))
    {
      return false;
    }

    // Step 5: Convert the output trajectory into a ROS-formatted message
    res.trajectory.header.stamp = ros::Time::now();
    res.trajectory.header.frame_id = "world";
    res.trajectory.joint_names = getJointNames();
    descartes_utilities::toRosJointPoints(*model_, result, 1.0, res.trajectory.points);
    return true;
  }

  EigenSTL::vector_Isometry3d makePuzzleToolPoses()
  {
    EigenSTL::vector_Isometry3d path;
    std::ifstream indata;

    std::string packagePath = ros::package::getPath("myworkcell_core");
    std::string filename = packagePath + "/config/puzzle_bent.csv";

    indata.open(filename);

    std::string line;
    int lnum = 0;
    while (std::getline(indata, line))
    {
        ++lnum;
        if (lnum < 3)
          continue;

        std::stringstream lineStream(line);
        std::string  cell;
        Eigen::VectorXd xyzijk(6);
        int i = -2;
        while (std::getline(lineStream, cell, ','))
        {
          ++i;
          if (i == -1)
            continue;

          xyzijk(i) = std::stod(cell);
        }

        Eigen::Vector3d pos = xyzijk.head<3>();
        pos = pos / 1000.0;
        Eigen::Vector3d norm = xyzijk.tail<3>();
        norm.normalize();

        Eigen::Vector3d temp_x = (-1 * pos).normalized();
        Eigen::Vector3d y_axis = (norm.cross(temp_x)).normalized();
        Eigen::Vector3d x_axis = (y_axis.cross(norm)).normalized();
        Eigen::Isometry3d pose;
        pose.matrix().col(0).head<3>() = x_axis;
        pose.matrix().col(1).head<3>() = y_axis;
        pose.matrix().col(2).head<3>() = norm;
        pose.matrix().col(3).head<3>() = pos;

        path.push_back(pose);
    }
    indata.close();

    return path;
  }

  bool visualizePuzzlePath(EigenSTL::vector_Isometry3d path)
  {
    int cnt = 0;
    visualization_msgs::MarkerArray marker_array;
    for (auto &point : path)
    {
      Eigen::Vector3d pos = point.matrix().col(3).head<3>();
      Eigen::Vector3d dir = point.matrix().col(2).head<3>();

      visualization_msgs::Marker marker;
      marker.header.frame_id = "part";
      marker.header.stamp = ros::Time();
      marker.header.seq = cnt;
      marker.ns = "markers";
      marker.id = cnt;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD;
      marker.lifetime = ros::Duration(0);
      marker.frame_locked = true;
      marker.scale.x = 0.0002;
      marker.scale.y = 0.0002;
      marker.scale.z = 0.0002;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      geometry_msgs::Point pnt1, pnt2;
      tf::pointEigenToMsg(pos, pnt1);
      tf::pointEigenToMsg(pos + (0.005*dir), pnt2);
      marker.points.push_back(pnt1);
      marker.points.push_back(pnt2);
      marker_array.markers.push_back(marker);
      ++cnt;
    }
    vis_pub_.publish(marker_array);
  }

  std::vector<descartes_core::TrajectoryPtPtr>
  makeDescartesTrajectory(const EigenSTL::vector_Isometry3d& path)
  {
    using namespace descartes_core;
    using namespace descartes_trajectory;

    std::vector<descartes_core::TrajectoryPtPtr> descartes_path; // return value

    // need to get the transform between grinder_frame and base_link;
    tf::StampedTransform grinder_frame;
    Eigen::Isometry3d gf;
    listener_.lookupTransform("world", "grinder_frame", ros::Time(0), grinder_frame);
    tf::transformTFToEigen(grinder_frame, gf);

    Frame wobj_base(gf);
    Frame tool_base = Frame::Identity();
    TolerancedFrame wobj_pt = Frame::Identity();

    for (auto& point : path)
    {
      auto p = point;
      TolerancedFrame tool_pt(p);
      tool_pt.orientation_tolerance.z_lower = -M_PI;
      tool_pt.orientation_tolerance.z_upper = +M_PI;

      boost::shared_ptr<CartTrajectoryPt> pt(new CartTrajectoryPt(wobj_base, wobj_pt, tool_base, tool_pt, 0, M_PI/20.0));
      descartes_path.push_back(pt);
    }
    return descartes_path;
  }

  // HELPER
  std::vector<std::string> getJointNames()
  {
    std::vector<std::string> names;
    nh_.getParam("controller_joint_names", names);
    return names;
  }


  boost::shared_ptr<ur5_demo_descartes::UR5RobotModel> model_;
  descartes_planner::DensePlanner planner_;
  ros::ServiceServer server_;
  ros::NodeHandle nh_;
  tf::TransformListener listener_;
  ros::Publisher vis_pub_;
};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "adv_descartes_node");

  ros::NodeHandle nh;
  CartesianPlanner planner (nh);

  ROS_INFO("Cartesian planning node starting");
  ros::spin();
}
