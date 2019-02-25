#include <tesseract_core/basic_types.h>
#include <tesseract_planning/trajopt/trajopt_planner.h>
#include <tesseract_ros/kdl/kdl_env.h>
#include <tesseract_ros/ros_basic_plotting.h>

#include <urdf_parser/urdf_parser.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <test_bed_core/trajopt_pick_and_place_constructor.h>
#include <test_bed_core/trajopt_utils.h>
#include <pick_and_place_perception/GetTargetPose.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <iiwa_msgs/JointPosition.h>

#include <trajopt/file_write_callback.hpp>
#include <trajopt/plot_callback.hpp>

int main(int argc, char** argv)
{
  //////////////////////
  /// INITIALIZATION ///
  //////////////////////

  ros::init(argc, argv, "test_bed_core_node");
  ros::NodeHandle nh, pnh("~");

  int steps_per_phase;
  std::string world_frame, pick_frame;
  bool sim_robot, actuate_gripper, plotting_cb, file_write_cb;

  pnh.param<int>("steps_per_phase", steps_per_phase, 10);
  nh.param<std::string>("world_frame", world_frame, "world");
  nh.param<std::string>("pick_frame", pick_frame, "part");
  pnh.param<bool>("sim_robot", sim_robot, true);
  pnh.param<bool>("actuate_gripper", actuate_gripper, true);
  pnh.param<bool>("plotting", plotting_cb, false);
  pnh.param<bool>("file_write_cb", file_write_cb, false);

  tf::TransformListener listener;
  ros::ServiceClient find_pick_client = nh.serviceClient<pick_and_place_perception::GetTargetPose>("find_pick");
  ros::Publisher test_pub = nh.advertise<trajectory_msgs::JointTrajectory>("joint_traj", 10);

  bool plan = true;

  // Set Log Level
  util::gLogLevel = util::LevelInfo;

  /////////////
  /// SETUP ///
  /////////////

  // Initial setup
  std::string urdf_xml_string, srdf_xml_string;
  nh.getParam("robot_description", urdf_xml_string);
  nh.getParam("robot_description_semantic", srdf_xml_string);

  urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF(urdf_xml_string);

  srdf::ModelSharedPtr srdf_model = srdf::ModelSharedPtr(new srdf::Model);
  srdf_model->initString(*urdf_model, srdf_xml_string);
  tesseract::tesseract_ros::KDLEnvPtr env(new tesseract::tesseract_ros::KDLEnv);
  assert(urdf_model != nullptr);
  assert(env != nullptr);

  bool success = env->init(urdf_model, srdf_model);
  assert(success);

  std::unordered_map<std::string, double> joint_states;

  // The initial robot position
  if (sim_robot)
  {
    joint_states["iiwa_joint_1"] = 0.0;
    joint_states["iiwa_joint_2"] = 0.0;
    joint_states["iiwa_joint_3"] = 0.0;
    joint_states["iiwa_joint_4"] = -1.57;
    joint_states["iiwa_joint_5"] = 0.0;
    joint_states["iiwa_joint_6"] = 0.0;
    joint_states["iiwa_joint_7"] = 0.0;
  }
  else
  {
    boost::shared_ptr<const iiwa_msgs::JointPosition> joint_pos =
        ros::topic::waitForMessage<iiwa_msgs::JointPosition>("iiwa/state/JointPosition", nh);

    joint_states["iiwa_joint_1"] = joint_pos.get()->position.a1;
    joint_states["iiwa_joint_2"] = joint_pos.get()->position.a2;
    joint_states["iiwa_joint_3"] = joint_pos.get()->position.a3;
    joint_states["iiwa_joint_4"] = joint_pos.get()->position.a4;
    joint_states["iiwa_joint_5"] = joint_pos.get()->position.a5;
    joint_states["iiwa_joint_6"] = joint_pos.get()->position.a6;
    joint_states["iiwa_joint_7"] = joint_pos.get()->position.a7;
  }
  env->setState(joint_states);

  double box_side, box_x, box_y;
  nh.getParam("box_x", box_x);
  nh.getParam("box_y", box_y);
  std::string box_parent_link;
  nh.getParam("box_parent_link", box_parent_link);

  ////////////
  /// PICK ///
  ////////////

  Eigen::Isometry3d world_to_box;
  pick_and_place_perception::GetTargetPose srv;
  double box_size_x, box_size_y, box_size_z;
  ROS_INFO("Calling Service to find pick location");
  // This calls the perception service
  if (find_pick_client.call(srv))
  {
    tf::poseMsgToEigen(srv.response.target_pose, world_to_box);
    plan &= srv.response.succeeded;
    box_size_x = srv.response.max_pt.x - srv.response.min_pt.x;
    box_size_y = srv.response.max_pt.y - srv.response.min_pt.y;
    box_size_z = srv.response.max_pt.z - 0.77153;  // Subtract off table height
  }
  else
  {
    ROS_ERROR("Failed to find pick location");
    plan = false;
  }

  // attach the simulated box in correct location
  tesseract::AttachableObjectPtr obj(new tesseract::AttachableObject());
  std::shared_ptr<shapes::Box> box(new shapes::Box());
  Eigen::Isometry3d box_pose = Eigen::Isometry3d::Identity();

  box->size[0] = box_size_x;
  box->size[1] = box_size_y;
  box->size[2] = box_size_z;

  obj->name = "box";
  obj->visual.shapes.push_back(box);
  obj->visual.shape_poses.push_back(box_pose);
  obj->collision.shapes.push_back(box);
  obj->collision.shape_poses.push_back(box_pose);
  obj->collision.collision_object_types.push_back(tesseract::CollisionObjectType::UseShapeType);

  env->addAttachableObject(obj);

  tesseract::AttachedBodyInfo attached_body;
  Eigen::Isometry3d object_pose = Eigen::Isometry3d::Identity();
  object_pose = world_to_box;
  object_pose.translation() += Eigen::Vector3d(0, 0, -0.77153 - box_size_z / 2.0);  // convert to world frame
  attached_body.object_name = "box";
  attached_body.parent_link_name = box_parent_link;
  attached_body.transform = object_pose;

  env->attachBody(attached_body);

  tesseract::tesseract_ros::ROSBasicPlotting plotter(env);
  Eigen::RowVectorXd init_pos = env->getCurrentJointValues();
  plotter.plotTrajectory(env->getJointNames(), init_pos.leftCols(env->getJointNames().size()));

  // Begin the path planning
  if (plan == true)
  {
    ROS_ERROR("Press enter to continue");
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    tesseract::tesseract_planning::TrajOptPlanner planner;
    tesseract::tesseract_planning::PlannerResponse planning_response;

    Eigen::Quaterniond orientation(0.0, 0.0, 1.0, 0.0);

    std::string manip = "Manipulator";
    std::string end_effector = "iiwa_link_ee";
    TrajoptPickAndPlaceConstructor prob_constructor(env, manip, end_effector, "box");

    // Define the final pose
    Eigen::Isometry3d final_pose;
    final_pose.linear() = orientation.matrix();
    final_pose.translation() = world_to_box.translation();
    double gripper_offset = 0.08;
    final_pose.translation() += Eigen::Vector3d(0.0, 0.0, gripper_offset);  // We add an offset for a gripper since it's not in the URDF

    // Define the approach pose
    Eigen::Isometry3d approach_pose = final_pose;
    approach_pose.translation() += Eigen::Vector3d(0.0, 0.0, 0.15);

    // Create and solve pick problem
    trajopt::TrajOptProbPtr pick_prob =
        prob_constructor.generatePickProblem(approach_pose, final_pose, steps_per_phase);

    // Set the parameters
    trajopt::BasicTrustRegionSQPParameters params;
    params.max_iter = 500;

    // Define Callbacks
    std::vector<trajopt::Optimizer::Callback> callbacks;

    // Create Plot Callback
    if (plotting_cb)
    {
      tesseract::tesseract_ros::ROSBasicPlottingPtr plotter_ptr(new tesseract::tesseract_ros::ROSBasicPlotting(env));
      callbacks.push_back(PlotCallback(*pick_prob, plotter_ptr));
    }
    // Create file write callback discarding any of the file's current contents
    std::shared_ptr<std::ofstream> stream_ptr(new std::ofstream);
    if (file_write_cb)
    {
      std::string path = ros::package::getPath("pick_and_place") + "/file_output_pick.csv";
      stream_ptr->open(path, std::ofstream::out | std::ofstream::trunc);
      callbacks.push_back(trajopt::WriteCallback(stream_ptr, pick_prob));
    }

    // Solve problem
    planner.solve(planning_response, pick_prob, params, callbacks);

    if (file_write_cb)
      stream_ptr->close();

    plotter.plotTrajectory(env->getJointNames(), planning_response.trajectory.leftCols(env->getJointNames().size()));
    std::cout << planning_response.trajectory << '\n';

    // Get transform b/n world and the parent link of the box transform
    tf::StampedTransform world_to_box_parent_link_tf;
    listener.lookupTransform(world_frame, box_parent_link, ros::Time(0.0), world_to_box_parent_link_tf);
    Eigen::Isometry3d world_to_box_parent_link;
    tf::transformTFToEigen(world_to_box_parent_link_tf, world_to_box_parent_link);

    Eigen::Isometry3d world_to_actual_box = world_to_box_parent_link;
    world_to_actual_box.translation() += Eigen::Vector3d(box_x, box_y, box_size_z);

    Eigen::Vector3d translation_err = (world_to_actual_box.inverse() * world_to_box).translation();

    // Publish the trajectory for debugging
    trajectory_msgs::JointTrajectory traj_msg3;
    ros::Duration t1(0.25);
    traj_msg3 = trajArrayToJointTrajectoryMsg(planning_response.joint_names, planning_response.trajectory, false, t1);
    test_pub.publish(traj_msg3);

    ROS_ERROR("Press enter to continue");
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    // detach the simulated box from the world and attach to the end effector
    env->detachBody("box");

    attached_body.parent_link_name = end_effector;
    attached_body.transform.translation() = Eigen::Vector3d(0, 0, box_size_z / 2.0 + gripper_offset);
    attached_body.touch_links = { "iiwa_link_ee", end_effector };  // allow the box to contact the end effector
    attached_body.touch_links = { "workcell_base",
                                  end_effector };  // allow the box to contact the table (since it's sitting on it)

    env->attachBody(attached_body);

    /////////////
    /// PLACE ///
    /////////////

    // Set the current state to the last state of the trajectory
    env->setState(env->getJointNames(), planning_response.trajectory.bottomRows(1).transpose());

    // Pick up box
    Eigen::Isometry3d retreat_pose = approach_pose;

    // Define some place locations.
    Eigen::Isometry3d middle_right_shelf, middle_left_shelf, top_right_shelf, top_left_shelf;
    middle_right_shelf.linear() = Eigen::Quaterniond(0, 0, 0.7071068, 0.7071068).matrix();
    middle_right_shelf.translation() = Eigen::Vector3d(0.148856, 0.73085 - gripper_offset, 1.16);
    middle_left_shelf.linear() = Eigen::Quaterniond(0, 0, 0.7071068, 0.7071068).matrix();
    middle_left_shelf.translation() = Eigen::Vector3d(-0.148856, 0.73085 - gripper_offset, 1.16);
    top_right_shelf.linear() = Eigen::Quaterniond(0, 0, 0.7071068, 0.7071068).matrix();
    top_right_shelf.translation() = Eigen::Vector3d(0.148856, 0.73085 - gripper_offset, 1.414);
    top_left_shelf.linear() = Eigen::Quaterniond(0, 0, 0.7071068, 0.7071068).matrix();
    top_left_shelf.translation() = Eigen::Vector3d(-0.148856, 0.73085 - gripper_offset, 1.414);

    // Set the target pose to middle_right_shelf
    final_pose = middle_left_shelf;

    // Setup approach for place
    approach_pose = final_pose;
    approach_pose.translation() += Eigen::Vector3d(0.0, -0.2, 0);

    // generate and solve the problem
    tesseract::tesseract_planning::PlannerResponse planning_response_place;
    trajopt::TrajOptProbPtr place_prob =
        prob_constructor.generatePlaceProblem(retreat_pose, approach_pose, final_pose, steps_per_phase);

    // Set the parameters
    tesseract::tesseract_planning::TrajOptPlannerConfig config_place(place_prob);
    config_place.params.max_iter = 500;

    // Create Plot Callback (optional)
    if (plotting_cb)
    {
      tesseract::tesseract_ros::ROSBasicPlottingPtr plotter_ptr(new tesseract::tesseract_ros::ROSBasicPlotting(env));
      config_place.callbacks.push_back(PlotCallback(*place_prob, plotter_ptr));
    }
    // Create file write callback discarding any of the file's current contents (optional)
    std::shared_ptr<std::ofstream> stream_ptr_place(new std::ofstream);
    if (file_write_cb)
    {
      std::string path = ros::package::getPath("pick_and_place") + "/file_output_place.csv";
      stream_ptr->open(path, std::ofstream::out | std::ofstream::trunc);
      config_place.callbacks.push_back(trajopt::WriteCallback(stream_ptr_place, place_prob));
    }

    // Solve problem
    planner.solve(planning_response_place, config_place);

    if (file_write_cb)
      stream_ptr_place->close();

    // plot the trajectory in Rviz
    plotter.plotTrajectory(env->getJointNames(),
                           planning_response_place.trajectory.leftCols(env->getJointNames().size()));
    std::cout << planning_response_place.trajectory << '\n';

    // Publish the trajectory for debugging
    trajectory_msgs::JointTrajectory traj_msg4;
    ros::Duration t2(0.25);
    traj_msg4 = trajArrayToJointTrajectoryMsg(
        planning_response_place.joint_names, planning_response_place.trajectory, false, t2);
    test_pub.publish(traj_msg4);

    ///////////////
    /// EXECUTE ///
    ///////////////

    // Execute on hardware
    if (!sim_robot)
    {
      // Execute trajectory
      std::cout << "Execute Trajectory on hardware? y/n \n";
      char input = 'n';
      std::cin >> input;
      if (input == 'y')
      {
        // Put gripper setup here (See Demo 3.10)
        if (actuate_gripper)
        {
        }  // End gripper setup here
        std::cout << "Executing... \n";

        // Create action client to send trajectories
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> execution_client("iiwa/"
                                                                                                  "PositionJointInterfa"
                                                                                                  "ce_trajectory_"
                                                                                                  "controller/"
                                                                                                  "follow_joint_"
                                                                                                  "trajectory",
                                                                                                  true);
        execution_client.waitForServer();

        // Convert TrajArray (Eigen Matrix of joint values) to ROS message
        trajectory_msgs::JointTrajectory traj_msg;
        ros::Duration t(0.25);
        traj_msg = trajArrayToJointTrajectoryMsg(planning_response.joint_names, planning_response.trajectory, false, t);

        // Create action message
        control_msgs::FollowJointTrajectoryGoal trajectory_action;
        trajectory_action.trajectory = traj_msg;
        //        trajectory_action.trajectory.header.frame_id="world";
        //        trajectory_action.trajectory.header.stamp = ros::Time(0);
        //        trajectory_action.goal_time_tolerance = ros::Duration(1.0);
        // May need to update other tolerances as well.

        // Send to hardware
        execution_client.sendGoal(trajectory_action);
        execution_client.waitForResult(ros::Duration(20.0));

        if (execution_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          std::cout << "Pick action succeeded! \n";
          // Put gripper pick code here (See Demo 3.10)
          if (actuate_gripper)
          {
          }  // End gripper pick code
        }
        else
        {
          std::cout << "Pick action failed \n";
        }
        std::cout << "Press enter to continue \n";
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        // Convert TrajArray (Eigen Matrix of joint values) to ROS message
        trajectory_msgs::JointTrajectory traj_msg2;
        ros::Duration t2(0.25);

        traj_msg2 = trajArrayToJointTrajectoryMsg(
            planning_response_place.joint_names, planning_response_place.trajectory, false, t2);
        test_pub.publish(traj_msg2);

        // Create action message
        control_msgs::FollowJointTrajectoryGoal trajectory_action_place;
        trajectory_action_place.trajectory = traj_msg2;
        // May need to update tolerances as well.

        // Send to hardware
        execution_client.sendGoal(trajectory_action_place);
        execution_client.waitForResult(ros::Duration(20.0));
        if (execution_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          std::cout << "Place action succeeded! \n";
          // Put gripper release code here (See Demo 3.10)
          if (actuate_gripper)
          {
          }  // End gripper release code
        }
        else
        {
          std::cout << "Place action failed \n";
        }
      }
    }
  }
  else
  {
    ROS_INFO("Planning disabled");
  }
  ROS_INFO("Done");
  ros::spin();
}
