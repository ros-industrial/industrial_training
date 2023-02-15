#include "planner_profiles.hpp"
#include "taskflow_generators.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tesseract_process_managers/core/process_planning_server.h>
#include <tesseract_motion_planners/default_planner_namespaces.h>
#include <tesseract_process_managers/task_profiles/check_input_profile.h>
#include <tesseract_process_managers/task_profiles/seed_min_length_profile.h>
#include <tesseract_process_managers/task_profiles/interative_spline_parameterization_profile.h>
#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_monitoring/environment_monitor_interface.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_geometry/mesh_parser.h>
#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/utils/utils.h>
#include <tesseract_rosutils/utils.h>
#include <snp_msgs/srv/generate_motion_plan.hpp>
#include <tf2_eigen/tf2_eigen.h>

static const std::string TRANSITION_PLANNER = "TRANSITION";
static const std::string FREESPACE_PLANNER = "FREESPACE";
static const std::string RASTER_PLANNER = "RASTER";
static const std::string PROFILE = "SNPD";
static const std::string PLANNING_SERVICE = "create_motion_plan";
static const std::string TESSERACT_MONITOR_NAMESPACE = "snp_environment";
static const double MAX_TCP_SPEED = 0.25;  // m/s

tesseract_common::Toolpath fromMsg(const snp_msgs::msg::ToolPaths& msg)
{
  tesseract_common::Toolpath tps;
  tps.reserve(msg.paths.size());
  for (const auto& path : msg.paths)
  {
    for (const auto& segment : path.segments)
    {
      tesseract_common::VectorIsometry3d seg;
      seg.reserve(segment.poses.size());
      for (const auto& pose : segment.poses)
      {
        Eigen::Isometry3d p;
        tf2::fromMsg(pose, p);

        // Rotate the pose 180 degrees about the x-axis such that the z-axis faces into the part
        p *= Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());

        seg.push_back(p);
      }
      tps.push_back(seg);
    }
  }
  return tps;
}

template <typename T>
T get(rclcpp::Node::SharedPtr node, const std::string& key)
{
  node->declare_parameter(key);
  T val;
  if (!node->get_parameter(key, val))
    throw std::runtime_error("Failed to get '" + key + "' parameter");
  return val;
}

static tesseract_environment::Commands createScanAdditionCommands(const std::string& filename,
                                                                  const std::string& mesh_frame,
                                                                  const std::vector<std::string>& touch_links)
{
  std::vector<tesseract_geometry::ConvexMesh::Ptr> geometries =
      tesseract_geometry::createMeshFromPath<tesseract_geometry::ConvexMesh>(filename);

  tesseract_scene_graph::Link link("scan");
  for (tesseract_geometry::ConvexMesh::Ptr geometry : geometries)
  {
    tesseract_scene_graph::Collision::Ptr collision = std::make_shared<tesseract_scene_graph::Collision>();
    collision->geometry = geometry;
    link.collision.push_back(collision);
  }

  tesseract_scene_graph::Joint joint("world_to_scan");
  joint.type = tesseract_scene_graph::JointType::FIXED;
  joint.parent_link_name = mesh_frame;
  joint.child_link_name = link.getName();

  tesseract_environment::Commands cmds;
  cmds.push_back(std::make_shared<tesseract_environment::AddLinkCommand>(link, joint, true));
  std::transform(touch_links.begin(), touch_links.end(), std::back_inserter(cmds),
                 [&link](const std::string& touch_link) {
                   return std::make_shared<tesseract_environment::AddAllowedCollisionCommand>(
                       touch_link, link.getName(), "USER_DEFINED");
                 });

  return cmds;
}

class PlanningServer
{
public:
  PlanningServer(rclcpp::Node::SharedPtr node)
    : node_(node)
    , verbose_(get<bool>(node_, "verbose"))
    , touch_links_(get<std::vector<std::string>>(node_, "touch_links"))
    , env_(std::make_shared<tesseract_environment::Environment>())
    , planning_server_(std::make_shared<tesseract_planning::ProcessPlanningServer>(env_))
  {
    {
      auto urdf_string = get<std::string>(node_, "robot_description");
      auto srdf_string = get<std::string>(node_, "robot_description_semantic");
      auto resource_locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
      if (!env_->init(urdf_string, srdf_string, resource_locator))
        throw std::runtime_error("Failed to initialize environment");
    }

    // Create the plotter
    plotter_ = std::make_shared<tesseract_rosutils::ROSPlotting>(env_->getRootLinkName());

    // Create the environment monitor
    tesseract_monitor_ =
        std::make_shared<tesseract_monitoring::EnvironmentMonitor>(node_, env_, TESSERACT_MONITOR_NAMESPACE);
    tesseract_monitor_->startStateMonitor(tesseract_monitoring::DEFAULT_JOINT_STATES_TOPIC, false);

    // Register custom process planners
    planning_server_->registerProcessPlanner(TRANSITION_PLANNER, createTransitionTaskflow());
    planning_server_->registerProcessPlanner(FREESPACE_PLANNER, createFreespaceTaskflow());
    planning_server_->registerProcessPlanner(RASTER_PLANNER, createRasterTaskflow());

    // Add custom profiles
    {
      auto pd = planning_server_->getProfiles();
      pd->addProfile<tesseract_planning::SimplePlannerPlanProfile>(
          tesseract_planning::profile_ns::SIMPLE_DEFAULT_NAMESPACE, PROFILE, createSimplePlannerProfile());
      pd->addProfile<tesseract_planning::OMPLPlanProfile>(tesseract_planning::profile_ns::OMPL_DEFAULT_NAMESPACE,
                                                          PROFILE, createOMPLProfile());
      pd->addProfile<tesseract_planning::TrajOptPlanProfile>(tesseract_planning::profile_ns::TRAJOPT_DEFAULT_NAMESPACE,
                                                             PROFILE, createTrajOptToolZFreePlanProfile());
      pd->addProfile<tesseract_planning::TrajOptCompositeProfile>(
          tesseract_planning::profile_ns::TRAJOPT_DEFAULT_NAMESPACE, PROFILE, createTrajOptProfile());
      pd->addProfile<tesseract_planning::DescartesPlanProfile<float>>(
          tesseract_planning::profile_ns::DESCARTES_DEFAULT_NAMESPACE, PROFILE, createDescartesPlanProfile<float>());
      pd->addProfile<tesseract_planning::CheckInputProfile>(
          tesseract_planning::profile_ns::CHECK_INPUT_DEFAULT_NAMESPACE, PROFILE,
          std::make_shared<tesseract_planning::CheckInputProfile>());
      pd->addProfile<tesseract_planning::SeedMinLengthProfile>(
          tesseract_planning::profile_ns::SEED_MIN_LENGTH_DEFAULT_NAMESPACE, PROFILE,
          std::make_shared<tesseract_planning::SeedMinLengthProfile>(5));
      pd->addProfile<tesseract_planning::IterativeSplineParameterizationProfile>(
          tesseract_planning::profile_ns::ITERATIVE_SPLINE_PARAMETERIZATION_DEFAULT_NAMESPACE, PROFILE,
          std::make_shared<tesseract_planning::IterativeSplineParameterizationProfile>());
    }

    // Advertise the ROS2 service
    server_ = node_->create_service<snp_msgs::srv::GenerateMotionPlan>(
        PLANNING_SERVICE, std::bind(&PlanningServer::plan, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(node_->get_logger(), "Started SNP motion planning server");
  }

private:
  tesseract_planning::CompositeInstruction createProgram(const tesseract_planning::ManipulatorInfo& info,
                                                         const tesseract_common::Toolpath& raster_strips)
  {
    std::vector<std::string> joint_names = env_->getJointGroup(info.manipulator)->getJointNames();

    tesseract_planning::CompositeInstruction program(PROFILE, tesseract_planning::CompositeInstructionOrder::ORDERED,
                                                     info);

    // Perform a freespace move to the first waypoint
    tesseract_planning::StateWaypoint swp1(joint_names, env_->getCurrentJointValues(joint_names));
    tesseract_planning::PlanInstruction start_instruction(swp1, tesseract_planning::PlanInstructionType::START,
                                                          PROFILE);
    program.setStartInstruction(start_instruction);

    for (std::size_t rs = 0; rs < raster_strips.size(); ++rs)
    {
      if (rs == 0)
      {
        // Define from start composite instruction
        tesseract_planning::CartesianWaypoint wp1 = raster_strips[rs][0];
        tesseract_planning::PlanInstruction plan_f0(wp1, tesseract_planning::PlanInstructionType::FREESPACE, PROFILE);
        plan_f0.setDescription("from_start_plan");
        tesseract_planning::CompositeInstruction from_start(PROFILE);
        from_start.setDescription("from_start");
        from_start.push_back(plan_f0);
        program.push_back(from_start);
      }

      // Define raster
      tesseract_planning::CompositeInstruction raster_segment(PROFILE);
      raster_segment.setDescription("Raster #" + std::to_string(rs + 1));

      for (std::size_t i = 1; i < raster_strips[rs].size(); ++i)
      {
        tesseract_planning::CartesianWaypoint wp = raster_strips[rs][i];
        raster_segment.push_back(
            tesseract_planning::PlanInstruction(wp, tesseract_planning::PlanInstructionType::LINEAR, PROFILE));
      }
      program.push_back(raster_segment);

      if (rs < raster_strips.size() - 1)
      {
        // Add transition
        tesseract_planning::CartesianWaypoint twp = raster_strips[rs + 1].front();

        tesseract_planning::PlanInstruction transition_instruction1(
            twp, tesseract_planning::PlanInstructionType::FREESPACE, PROFILE);
        transition_instruction1.setDescription("Transition #" + std::to_string(rs + 1));

        tesseract_planning::CompositeInstruction transition(PROFILE);
        transition.setDescription("Transition #" + std::to_string(rs + 1));
        transition.push_back(transition_instruction1);

        program.push_back(transition);
      }
      else
      {
        // Add to end instruction
        tesseract_planning::PlanInstruction plan_f2(swp1, tesseract_planning::PlanInstructionType::FREESPACE, PROFILE);
        plan_f2.setDescription("to_end_plan");
        tesseract_planning::CompositeInstruction to_end(PROFILE);
        to_end.setDescription("to_end");
        to_end.push_back(plan_f2);
        program.push_back(to_end);
      }
    }

    return program;
  }

  tesseract_common::JointTrajectory tcpSpeedLimiter(const tesseract_common::JointTrajectory& input_trajectory,
                                                    const double max_speed, const std::string tcp)
  {
    // Extract objects needed for calculating FK
    tesseract_common::JointTrajectory output_trajectory = input_trajectory;
    tesseract_scene_graph::StateSolver::UPtr state_solver = env_->getStateSolver();

    // Find the adjacent waypoints that require the biggest speed reduction to stay under the max tcp speed
    double strongest_scaling_factor = 1.0;
    for (std::size_t i = 1; i < output_trajectory.size(); i++)
    {
      // Find the previous waypoint position in Cartesian space
      tesseract_scene_graph::SceneState prev_ss =
          state_solver->getState(output_trajectory[i - 1].joint_names, output_trajectory[i - 1].position);
      Eigen::Isometry3d prev_pose = prev_ss.link_transforms[tcp];

      // Find the current waypoint position in Cartesian space
      tesseract_scene_graph::SceneState curr_ss =
          state_solver->getState(output_trajectory[i].joint_names, output_trajectory[i].position);
      Eigen::Isometry3d curr_pose = curr_ss.link_transforms[tcp];

      // Calculate the average TCP velocity between these waypoints
      double dist_traveled = (curr_pose.translation() - prev_pose.translation()).norm();
      double time_to_travel = output_trajectory[i].time - output_trajectory[i - 1].time;
      double original_velocity = dist_traveled / time_to_travel;

      // If the velocity is over the max speed determine the scaling factor and update greatest seen to this point
      if (original_velocity > max_speed)
      {
        double current_needed_scaling_factor = max_speed / original_velocity;
        if (current_needed_scaling_factor < strongest_scaling_factor)
          strongest_scaling_factor = current_needed_scaling_factor;
      }
    }

    // Apply the strongest scaling factor to all trajectory points to maintain a smooth trajectory
    double total_time = 0;
    for (std::size_t i = 1; i < output_trajectory.size(); i++)
    {
      double original_time_diff = input_trajectory[i].time - input_trajectory[i - 1].time;
      double new_time_diff = original_time_diff / strongest_scaling_factor;
      double new_timestamp = total_time + new_time_diff;
      // Apply new timestamp
      output_trajectory[i].time = new_timestamp;
      // Scale joint velocity by the scaling factor
      output_trajectory[i].velocity = input_trajectory[i].velocity * strongest_scaling_factor;
      // Scale joint acceleartion by the scaling factor squared
      output_trajectory[i].acceleration =
          input_trajectory[i].acceleration * strongest_scaling_factor * strongest_scaling_factor;
      // Update the total running time of the trajectory up to this point
      total_time = new_timestamp;
    }
    return output_trajectory;
  }

  void plan(const snp_msgs::srv::GenerateMotionPlan::Request::SharedPtr req,
            snp_msgs::srv::GenerateMotionPlan::Response::SharedPtr res)
  {
    try
    {
      RCLCPP_INFO_STREAM(node_->get_logger(), "Received motion planning request");

      // Create a manipulator info and program from the service request
      const std::string& base_frame = req->tool_paths.paths.at(0).segments.at(0).header.frame_id;
      tesseract_planning::ManipulatorInfo manip_info(req->motion_group, base_frame, req->tcp_frame);

      tesseract_planning::ProcessPlanningRequest plan_req;
      plan_req.name = RASTER_PLANNER;
      plan_req.instructions = createProgram(manip_info, fromMsg(req->tool_paths));
      plan_req.env_state = env_->getState();
      plan_req.commands = createScanAdditionCommands(req->mesh_filename, req->mesh_frame, touch_links_);

      auto log_level = console_bridge::getLogLevel();
      if (verbose_)
        console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);

      tesseract_planning::ProcessPlanningFuture plan_result = planning_server_->run(plan_req);
      plan_result.wait();

      // Reset the log level
      console_bridge::setLogLevel(log_level);

      if (!plan_result.interface->isSuccessful())
        throw std::runtime_error("Failed to create motion plan");

      // Convert to joint trajectory
      tesseract_common::JointTrajectory jt =
          toJointTrajectory(plan_result.results->as<tesseract_planning::CompositeInstruction>());
      tesseract_common::JointTrajectory tcp_velocity_scaled_jt = tcpSpeedLimiter(jt, MAX_TCP_SPEED, "tool0");
      plotter_->plotTrajectory(tcp_velocity_scaled_jt, *env_->getStateSolver());
      res->motion_plan = tesseract_rosutils::toMsg(tcp_velocity_scaled_jt, env_->getState());

      res->message = "Succesfully planned motion";
      res->success = true;
    }
    catch (const std::exception& ex)
    {
      res->message = ex.what();
      res->success = false;
    }

    RCLCPP_INFO_STREAM(node_->get_logger(), res->message);
  }

  rclcpp::Node::SharedPtr node_;
  const bool verbose_{ false };
  const std::vector<std::string> touch_links_;
  tesseract_environment::Environment::Ptr env_;
  tesseract_planning::ProcessPlanningServer::Ptr planning_server_;
  tesseract_monitoring::EnvironmentMonitor::Ptr tesseract_monitor_;
  tesseract_rosutils::ROSPlotting::Ptr plotter_;
  rclcpp::Service<snp_msgs::srv::GenerateMotionPlan>::SharedPtr server_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("snp_planning_server");
  auto server = std::make_shared<PlanningServer>(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
}
