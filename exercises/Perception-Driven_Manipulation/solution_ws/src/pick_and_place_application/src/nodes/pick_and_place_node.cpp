
#include <pick_and_place_application/pick_and_place.h>

using namespace pick_and_place_application;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_and_place_node");

// =============================== Main Thread ===============================
int main(int argc,char** argv)
{
  geometry_msgs::msg::Pose box_pose;
  std::vector<geometry_msgs::msg::Pose> pick_poses, place_poses;

  /* =========================================================================================*/
  /*	INITIALIZING ROS NODE
      Goal:
      - Observe all steps needed to properly initialize a ros node.
      - Look into the 'cfg' member of PickAndPlace to take notice of the parameters that
        are available for the rest of the program. */
  /* =========================================================================================*/

  RCLCPP_INFO(LOGGER, "Initialize node");
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true); // This enables loading undeclared parameters
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("pick_and_place_node", "", node_options);

  // spinning node in a separate thread
  std::thread spin_thread([node](){
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  });

  // creating pick and place application instance
  PickAndPlaceApp application(node);

  // initializing application
  if(!application.initialize())
  {
    return false;
  }

  /* ========================================*/
  /* Pick & Place Tasks                      */
  /* ========================================*/

  // move to a "clear" position
  application.move_to_wait_position();

  // turn off vacuum gripper
  application.set_gripper(false);

  // get the box position and orientation
  box_pose = application.detect_box_pick();

  // build a sequence of poses to "pick" the box
  pick_poses = application.create_pick_moves(box_pose);

  // plan/execute the sequence of "pick" moves
  application.pickup_box(pick_poses,box_pose);

  // build a sequence of poses to "place" the box
  place_poses = application.create_place_moves();

  // plan/execute the "place" moves
  application.place_box(place_poses,box_pose);

  // move back to the "clear" position
  application.move_to_wait_position();

  rclcpp::shutdown();
  spin_thread.join();

  return 0;
}
