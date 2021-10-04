
#include <pick_and_place_application/pick_and_place.h>

using namespace pick_and_place_application;

// create a logger to print messages into the terminal
static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_and_place_node");

// =============================== Main Thread ===============================
int main(int argc, char** argv)
{
  geometry_msgs::msg::Pose box_pose;
  std::vector<geometry_msgs::msg::Pose> pick_poses, place_poses;

  /* =========================================================================================*/
  /*	INITIALIZING ROS NODE
      Goal:
      - Observe all steps needed to properly initialize a ros node.
      - Take notice of how the node's spin() method is invoked from within a thread.  This avoids locking the application.
  /* =========================================================================================*/

  RCLCPP_INFO(LOGGER, "Initialize node");
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);  // This enables loading undeclared parameters
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("pick_and_place_node", "", node_options);

  // spinning node in a separate thread
  std::thread spin_thread([node]() {
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  });

  // creating pick and place application instance
  PickAndPlaceApp application(node);

  // initializing application
  application.initialize();

  /* ========================================*/
  /* Pick & Place Tasks                      */
  /* ========================================*/

  // reset world
  application.resetWorld(false);

  // move to a "clear" position
  application.moveToWaitPosition();

  // turn off vacuum gripper
  application.actuateGripper(false);

  // get the box position and orientation
  box_pose = application.detectBox();

  // build a sequence of poses to "pick" the box
  pick_poses = application.computePickToolPoses(box_pose);

  // plan/execute the sequence of "pick" moves
  application.doBoxPickup(pick_poses, box_pose);

  // build a sequence of poses to "place" the box
  place_poses = application.computePlaceToolPoses();

  // plan/execute the "place" moves
  application.doBoxPlace(place_poses, box_pose);

  // move back to the "clear" position
  application.moveToWaitPosition();

  rclcpp::shutdown();
  spin_thread.join();

  return 0;
}
