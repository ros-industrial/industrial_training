#include "planner_profiles.hpp"
#include "plugins/tasks/constant_tcp_speed_time_parameterization_profile.hpp"
#include "plugins/tasks/kinematic_limits_check_profile.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tesseract_time_parameterization/isp/iterative_spline_parameterization.h>
#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_monitoring/environment_monitor_interface.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_geometry/mesh_parser.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_collision/bullet/convex_hull_utils.h>
#include <snp_msgs/srv/generate_motion_plan.hpp>
#include <tf2_eigen/tf2_eigen.h>

#include <tesseract_time_parameterization/core/instructions_trajectory.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/interface_utils.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/profile_dictionary.h>
#include <tesseract_command_language/utils.h>

#include <tesseract_task_composer/planning/planning_task_composer_problem.h>
#include <tesseract_task_composer/planning/profiles/min_length_profile.h>
#include <tesseract_task_composer/planning/profiles/iterative_spline_parameterization_profile.h>
#include <tesseract_task_composer/planning/profiles/contact_check_profile.h>

#include <tesseract_task_composer/core/task_composer_problem.h>
#include <tesseract_task_composer/core/task_composer_input.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>

static const std::string TRANSITION_PLANNER = "TRANSITION";
static const std::string FREESPACE_PLANNER = "FREESPACE";
static const std::string RASTER_PLANNER = "RASTER";
static const std::string PROFILE = "SNPD";
static const std::string PLANNING_SERVICE = "create_motion_plan";
static const std::string TESSERACT_MONITOR_NAMESPACE = "snp_environment";
static const double MAX_TCP_SPEED = 0.25;  // m/s

static const std::string VERBOSE_PARAM = "verbose";
static const std::string TOUCH_LINKS_PARAM = "touch_links";
static const std::string MAX_TRANS_VEL_PARAM = "max_translational_vel";
static const std::string MAX_ROT_VEL_PARAM = "max_rotational_vel";
static const std::string MAX_TRANS_ACC_PARAM = "max_translational_acc";
static const std::string MAX_ROT_ACC_PARAM = "max_rotational_acc";
static const std::string CHECK_JOINT_ACC_PARAM = "check_joint_accelerations";
static const std::string VEL_SCALE_PARAM = "velocity_scaling_factor";
static const std::string ACC_SCALE_PARAM = "acceleration_scaling_factor";
static const std::string LVS_PARAM = "contact_check_longest_valid_segment";
static const std::string CONTACT_DIST_PARAM = "contact_check_distance";

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
  T val;
  if (!node->get_parameter(key, val))
    throw std::runtime_error("Failed to get '" + key + "' parameter");
  return val;
}

double clamp(const double val, const double min, const double max)
{
  return std::min(std::max(val, min), max);
}

static tesseract_environment::Commands createScanAdditionCommands(const std::string& filename,
                                                                  const std::string& mesh_frame,
                                                                  const std::vector<std::string>& touch_links)
{
  std::vector<tesseract_geometry::Mesh::Ptr> geometries =
      tesseract_geometry::createMeshFromPath<tesseract_geometry::Mesh>(filename);

  tesseract_scene_graph::Link link("scan");
  for (tesseract_geometry::Mesh::Ptr geometry : geometries)
  {
    tesseract_scene_graph::Collision::Ptr collision = std::make_shared<tesseract_scene_graph::Collision>();
    collision->geometry = tesseract_collision::makeConvexMesh(*geometry);
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
                   tesseract_common::AllowedCollisionMatrix acm;
                   acm.addAllowedCollision(touch_link, link.getName(), "USER_DEFINED");
                   return std::make_shared<tesseract_environment::ModifyAllowedCollisionsCommand>(
                       acm, tesseract_environment::ModifyAllowedCollisionsType::ADD);
                 });

  return cmds;
}

class PlanningServer
{
public:
  PlanningServer(rclcpp::Node::SharedPtr node)
    : node_(node), env_(std::make_shared<tesseract_environment::Environment>())
  {
    // Declare ROS parameters
    node_->declare_parameter<std::string>("robot_description");
    node_->declare_parameter<std::string>("robot_description_semantic");
    node_->declare_parameter<bool>(VERBOSE_PARAM, false);
    node_->declare_parameter<std::vector<std::string>>(TOUCH_LINKS_PARAM, {});
    node_->declare_parameter<double>(MAX_TRANS_VEL_PARAM);
    node_->declare_parameter<double>(MAX_ROT_VEL_PARAM);
    node_->declare_parameter<double>(MAX_TRANS_ACC_PARAM);
    node_->declare_parameter<double>(MAX_ROT_ACC_PARAM);
    node_->declare_parameter<bool>(CHECK_JOINT_ACC_PARAM, false);
    node_->declare_parameter<double>(VEL_SCALE_PARAM, 1.0);
    node_->declare_parameter<double>(ACC_SCALE_PARAM, 1.0);
    node_->declare_parameter<double>(LVS_PARAM, 0.05);
    node_->declare_parameter<double>(CONTACT_DIST_PARAM, 0.0);

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
        std::make_shared<tesseract_monitoring::ROSEnvironmentMonitor>(node_, env_, TESSERACT_MONITOR_NAMESPACE);
    tesseract_monitor_->setEnvironmentPublishingFrequency(30.0);
    tesseract_monitor_->startPublishingEnvironment();
    tesseract_monitor_->startStateMonitor("/robot_joint_states", true);

    // Advertise the ROS2 service
    server_ = node_->create_service<snp_msgs::srv::GenerateMotionPlan>(
        PLANNING_SERVICE, std::bind(&PlanningServer::plan, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(node_->get_logger(), "Started SNP motion planning server");
  }

private:
  tesseract_planning::CompositeInstruction createProgram(const tesseract_common::ManipulatorInfo& info,
                                                         const tesseract_common::Toolpath& raster_strips)
  {
    std::vector<std::string> joint_names = env_->getJointGroup(info.manipulator)->getJointNames();

    tesseract_planning::CompositeInstruction program(PROFILE, tesseract_planning::CompositeInstructionOrder::ORDERED,
                                                     info);

    // Perform a freespace move to the first waypoint
    tesseract_planning::StateWaypoint swp1(joint_names, env_->getCurrentJointValues(joint_names));

    // Create object needed for move instruction
    tesseract_planning::StateWaypointPoly swp1_poly{ swp1 };

    tesseract_planning::MoveInstruction start_instruction(swp1_poly, tesseract_planning::MoveInstructionType::FREESPACE,
                                                          PROFILE, info);

    for (std::size_t rs = 0; rs < raster_strips.size(); ++rs)
    {
      if (rs == 0)
      {
        // Define from start composite instruction
        tesseract_planning::CartesianWaypoint wp1 = raster_strips[rs][0];
        tesseract_planning::CartesianWaypointPoly wp1_poly{ wp1 };
        tesseract_planning::MoveInstruction move_f0(wp1_poly, tesseract_planning::MoveInstructionType::FREESPACE,
                                                    PROFILE, info);
        move_f0.setDescription("from_start_plan");
        tesseract_planning::CompositeInstruction from_start(PROFILE);
        from_start.setDescription("from_start");
        from_start.appendMoveInstruction(start_instruction);
        from_start.appendMoveInstruction(move_f0);
        program.push_back(from_start);
      }

      // Define raster
      tesseract_planning::CompositeInstruction raster_segment(PROFILE);
      raster_segment.setDescription("Raster #" + std::to_string(rs + 1));

      for (std::size_t i = 1; i < raster_strips[rs].size(); ++i)
      {
        tesseract_planning::CartesianWaypoint wp = raster_strips[rs][i];
        tesseract_planning::CartesianWaypointPoly wp_poly{ wp };
        raster_segment.appendMoveInstruction(tesseract_planning::MoveInstruction(
            wp_poly, tesseract_planning::MoveInstructionType::LINEAR, PROFILE, info));
      }
      program.push_back(raster_segment);

      if (rs < raster_strips.size() - 1)
      {
        // Add transition
        tesseract_planning::CartesianWaypoint twp = raster_strips[rs + 1].front();
        tesseract_planning::CartesianWaypointPoly twp_poly{ twp };

        tesseract_planning::MoveInstruction transition_instruction1(
            twp_poly, tesseract_planning::MoveInstructionType::FREESPACE, PROFILE, info);
        transition_instruction1.setDescription("Transition #" + std::to_string(rs + 1));

        tesseract_planning::CompositeInstruction transition(PROFILE);
        transition.setDescription("Transition #" + std::to_string(rs + 1));
        transition.appendMoveInstruction(transition_instruction1);

        program.push_back(transition);
      }
      else
      {
        // Add to end instruction
        tesseract_planning::MoveInstruction plan_f2(swp1_poly, tesseract_planning::MoveInstructionType::FREESPACE,
                                                    PROFILE, info);
        plan_f2.setDescription("to_end_plan");
        tesseract_planning::CompositeInstruction to_end(PROFILE);
        to_end.setDescription("to_end");
        to_end.appendMoveInstruction(plan_f2);
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

      tesseract_planning::ProfileDictionary::Ptr profile_dict =
          std::make_shared<tesseract_planning::ProfileDictionary>();
      // Add custom profiles
      {
        profile_dict->addProfile<tesseract_planning::SimplePlannerPlanProfile>(SIMPLE_DEFAULT_NAMESPACE, PROFILE,
                                                                               createSimplePlannerProfile());
        profile_dict->addProfile<tesseract_planning::OMPLPlanProfile>(OMPL_DEFAULT_NAMESPACE, PROFILE,
                                                                      createOMPLProfile());
        profile_dict->addProfile<tesseract_planning::TrajOptPlanProfile>(TRAJOPT_DEFAULT_NAMESPACE, PROFILE,
                                                                         createTrajOptToolZFreePlanProfile());
        profile_dict->addProfile<tesseract_planning::TrajOptCompositeProfile>(TRAJOPT_DEFAULT_NAMESPACE, PROFILE,
                                                                              createTrajOptProfile());
        profile_dict->addProfile<tesseract_planning::DescartesPlanProfile<float>>(DESCARTES_DEFAULT_NAMESPACE, PROFILE,
                                                                                  createDescartesPlanProfile<float>());
        profile_dict->addProfile<tesseract_planning::MinLengthProfile>(
            MIN_LENGTH_DEFAULT_NAMESPACE, PROFILE, std::make_shared<tesseract_planning::MinLengthProfile>(6));
        auto velocity_scaling_factor =
            clamp(get<double>(node_, VEL_SCALE_PARAM), std::numeric_limits<double>::epsilon(), 1.0);
        auto acceleration_scaling_factor =
            clamp(get<double>(node_, ACC_SCALE_PARAM), std::numeric_limits<double>::epsilon(), 1.0);

        // ISP profile
        profile_dict->addProfile<tesseract_planning::IterativeSplineParameterizationProfile>(
            ISP_DEFAULT_NAMESPACE, PROFILE,
            std::make_shared<tesseract_planning::IterativeSplineParameterizationProfile>(velocity_scaling_factor,
                                                                                         acceleration_scaling_factor));

        // Discrete contact check profile
        auto contact_check_lvs = get<double>(node_, LVS_PARAM);
        auto contact_check_dist = get<double>(node_, CONTACT_DIST_PARAM);
        profile_dict->addProfile<tesseract_planning::ContactCheckProfile>(
            CONTACT_CHECK_DEFAULT_NAMESPACE, PROFILE,
            std::make_shared<tesseract_planning::ContactCheckProfile>(contact_check_lvs, contact_check_dist));

        // Constant TCP time parameterization profile
        auto vel_trans = get<double>(node_, MAX_TRANS_VEL_PARAM);
        auto vel_rot = get<double>(node_, MAX_ROT_VEL_PARAM);
        auto acc_trans = get<double>(node_, MAX_TRANS_ACC_PARAM);
        auto acc_rot = get<double>(node_, MAX_ROT_ACC_PARAM);
        auto cart_time_param_profile =
            std::make_shared<snp_motion_planning::ConstantTCPSpeedTimeParameterizationProfile>(
                vel_trans, vel_rot, acc_trans, acc_rot, velocity_scaling_factor, acceleration_scaling_factor);
        profile_dict->addProfile<snp_motion_planning::ConstantTCPSpeedTimeParameterizationProfile>(
            CONSTANT_TCP_SPEED_TIME_PARAM_TASK_NAME, PROFILE, cart_time_param_profile);

        // Kinematic limit check
        auto check_joint_acc = get<bool>(node_, CHECK_JOINT_ACC_PARAM);
        auto kin_limit_check_profile =
            std::make_shared<snp_motion_planning::KinematicLimitsCheckProfile>(true, true, check_joint_acc);
        profile_dict->addProfile<snp_motion_planning::KinematicLimitsCheckProfile>(KINEMATIC_LIMITS_CHECK_TASK_NAME,
                                                                                   PROFILE, kin_limit_check_profile);
      }

      // Create a manipulator info and program from the service request
      const std::string& base_frame = req->tool_paths.paths.at(0).segments.at(0).header.frame_id;
      tesseract_common::ManipulatorInfo manip_info(req->motion_group, base_frame, req->tcp_frame);

      // Set up composite instruction and environment
      tesseract_planning::CompositeInstruction program = createProgram(manip_info, fromMsg(req->tool_paths));
      tesseract_environment::Commands env_cmds = createScanAdditionCommands(
          req->mesh_filename, req->mesh_frame, get<std::vector<std::string>>(node_, TOUCH_LINKS_PARAM));
      tesseract_environment::Environment::Ptr planner_env = env_->clone();
      planner_env->applyCommands(env_cmds);

      // Set up task composer problem
      std::string config_path = ament_index_cpp::get_package_share_directory("snp_motion_planning");
      config_path += "/config/task_composer_plugins.yaml";
      tesseract_planning::TaskComposerPluginFactory factory(YAML::LoadFile(config_path));
      std::string task_pipeline = "SNPPipeline";
      auto executor = factory.createTaskComposerExecutor("TaskflowExecutor");
      tesseract_planning::TaskComposerNode::UPtr task = factory.createTaskComposerNode(task_pipeline);
      // Save dot graph
      std::ofstream tc_out_data;
      tc_out_data.open(tesseract_common::getTempPath() + "ScanNPlanPipeline.dot");
      task->dump(tc_out_data);
      const std::string input_key = task->getInputKeys().front();
      const std::string output_key = task->getOutputKeys().front();
      tesseract_planning::TaskComposerDataStorage input_data;
      input_data.setData(input_key, program);
      tesseract_planning::TaskComposerProblem::UPtr problem =
          std::make_unique<tesseract_planning::PlanningTaskComposerProblem>(planner_env, input_data, profile_dict);
      tesseract_planning::TaskComposerInput input(std::move(problem));
      input.dotgraph = true;

      // Update log level for debugging
      auto log_level = console_bridge::getLogLevel();
      if (get<bool>(node_, VERBOSE_PARAM))
      {
        console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
        // Create a dump dotgraphs of each task for reference
        std::ofstream cartesian_pipeline_out_data;
        cartesian_pipeline_out_data.open(tesseract_common::getTempPath() + "SNPCartesianPipeline.dot");
        factory.createTaskComposerNode("SNPCartesianPipeline")->dump(cartesian_pipeline_out_data);

        std::ofstream freespace_pipeline_out_data;
        freespace_pipeline_out_data.open(tesseract_common::getTempPath() + "SNPFreespacePipeline.dot");
        factory.createTaskComposerNode("SNPFreespacePipeline")->dump(freespace_pipeline_out_data);

        std::ofstream transition_pipeline_out_data;
        transition_pipeline_out_data.open(tesseract_common::getTempPath() + "SNPTransitionPipeline.dot");
        factory.createTaskComposerNode("SNPTransitionPipeline")->dump(transition_pipeline_out_data);
      }

      // Run problem
      tesseract_planning::TaskComposerFuture::UPtr exec_fut = executor->run(*task, input);
      exec_fut->wait();

      auto info_map = input.task_infos.getInfoMap();
      std::ofstream tc_out_results;
      tc_out_results.open(tesseract_common::getTempPath() + "ScanNPlanPipelineResults.dot");
      static_cast<const tesseract_planning::TaskComposerGraph&>(*task).dump(tc_out_results, nullptr, info_map);
      tc_out_results.close();

      // Reset the log level
      console_bridge::setLogLevel(log_level);

      // Check for successful plan
      if (!input.isSuccessful())
        throw std::runtime_error("Failed to create motion plan");

      // Get results of successful plan
      tesseract_planning::CompositeInstruction program_results =
          input.data_storage.getData(output_key).as<tesseract_planning::CompositeInstruction>();

      // Convert to joint trajectory
      tesseract_common::JointTrajectory jt = toJointTrajectory(program_results);
      tesseract_common::JointTrajectory tcp_velocity_scaled_jt = tcpSpeedLimiter(jt, MAX_TCP_SPEED, "tool0");

      // Send joint trajectory to Tesseract plotter widget
      plotter_->plotTrajectory(jt, *env_->getStateSolver());

      // Return results
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

  tesseract_environment::Environment::Ptr env_;
  tesseract_monitoring::ROSEnvironmentMonitor::Ptr tesseract_monitor_;
  tesseract_rosutils::ROSPlottingPtr plotter_;
  rclcpp::Service<snp_msgs::srv::GenerateMotionPlan>::SharedPtr server_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("snp_planning_server");
  auto server = std::make_shared<PlanningServer>(node);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
}
