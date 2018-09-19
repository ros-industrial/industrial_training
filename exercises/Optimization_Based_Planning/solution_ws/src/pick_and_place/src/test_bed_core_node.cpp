#include <tesseract_core/basic_types.h>
#include <tesseract_planning/trajopt/trajopt_planner.h>
#include <tesseract_ros/kdl/kdl_env.h>
#include <tesseract_ros/ros_basic_plotting.h>

#include <urdf_parser/urdf_parser.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <test_bed_core/trajopt_pick_and_place_constructor.h>
#include <pick_and_place_perception/GetTargetPose.h>

int main(int argc, char** argv)
{
  //////////////////////
  /// INITIALIZATION ///
  //////////////////////

  ros::init(argc, argv, "test_bed_core_node");
  ros::NodeHandle nh, pnh("~");

  int steps_per_phase;
  std::string world_frame, pick_frame;
  pnh.param<int>("steps_per_phase", steps_per_phase, 50);

  nh.param<std::string>("world_frame", world_frame, "world");
  nh.param<std::string>("pick_frame", pick_frame, "part");

  tf::TransformListener listener;
  ros::ServiceClient find_pick_client = nh.serviceClient<pick_and_place_perception::GetTargetPose>("find_pick");

  bool plan = true;

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
  joint_states["panda_joint1"] = 0.0;
  joint_states["panda_joint2"] = 0.0;
  joint_states["panda_joint3"] = 0.0;
  joint_states["panda_joint4"] = -1.57;
  joint_states["panda_joint5"] = 0.0;
  joint_states["panda_joint6"] = 0.0;
  joint_states["panda_joint7"] = 0.0;
  env->setState(joint_states);

  double box_side, box_x, box_y;
  nh.getParam("box_side", box_side);
  nh.getParam("box_x", box_x);
  nh.getParam("box_y", box_y);

  std::string box_parent_link;
  nh.getParam("box_parent_link", box_parent_link);

  // attach the simulated box
  tesseract::AttachableObjectPtr obj(new tesseract::AttachableObject());
  std::shared_ptr<shapes::Box> box(new shapes::Box());
  Eigen::Isometry3d box_pose = Eigen::Isometry3d::Identity();

  box->size[0] = box_side;
  box->size[1] = box_side;
  box->size[2] = box_side;

  obj->name = "box";
  obj->visual.shapes.push_back(box);
  obj->visual.shape_poses.push_back(box_pose);
  obj->collision.shapes.push_back(box);
  obj->collision.shape_poses.push_back(box_pose);
  obj->collision.collision_object_types.push_back(tesseract::CollisionObjectType::UseShapeType);

  env->addAttachableObject(obj);

  tesseract::AttachedBodyInfo attached_body;
  Eigen::Isometry3d object_pose = Eigen::Isometry3d::Identity();
  object_pose.translation() += Eigen::Vector3d(box_x, box_y, box_side / 2.0);
  attached_body.object_name = "box";
  attached_body.parent_link_name = box_parent_link;
  attached_body.transform = object_pose;

  env->attachBody(attached_body);

  tesseract::tesseract_ros::ROSBasicPlotting plotter(env);
  Eigen::VectorXd init_pos = env->getCurrentJointValues();
  init_pos.conservativeResize(init_pos.rows() + 1);
  plotter.plotTrajectory(env->getJointNames(), init_pos);

  ////////////
  /// PICK ///
  ////////////

  Eigen::Isometry3d world_to_box;
  pick_and_place_perception::GetTargetPose srv;
  ROS_INFO("Calling Service to find pick location");
  // This calls the perception service
  if (find_pick_client.call(srv))
  {
    tf::poseMsgToEigen(srv.response.target_pose, world_to_box);
    plan&=srv.response.succeeded;
  }
  else
  {
    ROS_ERROR("Failed to find pick location");
    plan = false;
  }

  if (plan == true)
  {
  ROS_ERROR("Press enter to continue");
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  tesseract::tesseract_planning::TrajoptPlanner planner;
  tesseract::tesseract_planning::PlannerResponse planning_response;

  Eigen::Quaterniond orientation(0.0, 0.0, 1.0, 0.0);

  std::string manip = "Manipulator";
  std::string end_effector = "panda_link8";
  TrajoptPickAndPlaceConstructor prob_constructor(env, manip, end_effector, "box");
  Eigen::Isometry3d final_pose;
  final_pose.linear() = orientation.matrix();
  final_pose.translation() = world_to_box.translation();

  Eigen::Isometry3d approach_pose = final_pose;
  approach_pose.translation() += Eigen::Vector3d(0.0, 0.0, 0.15);

  trajopt::TrajOptProbPtr pick_prob = prob_constructor.generatePickProblem(approach_pose, final_pose, steps_per_phase);
  planner.solve(pick_prob, planning_response);
  plotter.plotTrajectory(env->getJointNames(), planning_response.trajectory);

  tf::StampedTransform world_to_box_parent_link_tf;
  listener.lookupTransform(world_frame, box_parent_link, ros::Time(0.0), world_to_box_parent_link_tf);

  Eigen::Isometry3d world_to_box_parent_link;
  tf::transformTFToEigen(world_to_box_parent_link_tf, world_to_box_parent_link);

  Eigen::Isometry3d world_to_actual_box = world_to_box_parent_link;
  world_to_actual_box.translation() += Eigen::Vector3d(box_x, box_y, box_side);

  Eigen::Vector3d translation_err = (world_to_actual_box.inverse() * world_to_box).translation();

  ROS_ERROR("Press enter to continue");
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  /////////////
  /// PLACE ///
  /////////////

  // detach the simulated box from the world and attach to the end effector
  env->detachBody("box");

  attached_body.parent_link_name = end_effector;
  attached_body.transform.translation() = Eigen::Vector3d(translation_err.x(), translation_err.y(), box_side / 2.0);
  attached_body.touch_links = { "panda_link7", end_effector };  // allow the box to contact the end effector

  env->attachBody(attached_body);

  // Set the current state to the last state of the trajectory
  env->setState(
      env->getJointNames(),
      planning_response.trajectory.block(steps_per_phase * 2 - 1, 0, 1, env->getJointNames().size()).transpose());

  // create some arbitrary pose checkpoints and goals
  Eigen::Isometry3d retreat_pose = approach_pose;

  Eigen::Vector3d box_move(0.7, -0.1, 0.0);
  approach_pose.translation() += box_move;

  final_pose.translation() += box_move;

  // generate and solve the problem
  trajopt::TrajOptProbPtr place_prob =
      prob_constructor.generatePlaceProblem(retreat_pose, approach_pose, final_pose, steps_per_phase);
  planner.solve(place_prob, planning_response);

  // plot the trajectory in Rviz
  plotter.plotTrajectory(env->getJointNames(), planning_response.trajectory);

  // TODO send the trajectory to the robot

  // TODO execute place trajectory
  }
  else
  {
   ROS_INFO("Planning disabled");
  }
  ROS_INFO("Done");
  ros::spin();
}
