/*
 * initialize.cpp
 *
 *  Created on: Aug 13, 2021
 *      Author: jnicho
 */

#include <pick_and_place_application/pick_and_place.h>

/* INITIALIZE
  Goal:
    - Understand how parameters get loaded into the application with the application specific "cfg.init()" method.
    - Understand how to initialize the various ROS2 interfaces (actions, subscribers, publishers, services) used by this application.
*/

namespace pick_and_place_application
{
void PickAndPlaceApp::initialize()
{
  // reading parameters
  if (this->cfg.init(node))
  {
    RCLCPP_INFO(node->get_logger(), "Parameters successfully loaded");
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Parameters failed to load");
    throw std::runtime_error("Parameter lookup failure");
  }

  // marker publisher
  marker_publisher = node->create_publisher<visualization_msgs::msg::Marker>(cfg.MARKER_TOPIC, 1);

  // planning scene publisher
  planning_scene_publisher = node->create_publisher<moveit_msgs::msg::PlanningScene>(cfg.PLANNING_SCENE_TOPIC, 1);

  // target recognition client (perception)
  target_recognition_client = node->create_client<pick_and_place_msgs::srv::GetTargetPose>(cfg.GET_TARGET_POSE_SERVICE);

  // grasp action client (vacuum gripper)
  grasp_action_client = rclcpp_action::create_client<ExecuteGraspAction>(node, cfg.GRASP_ACTION_NAME);

  // instantiate the moveit cpp object for motion planing
  moveit_cpp = std::make_shared<moveit_cpp::MoveItCpp>(node);

  // waiting to establish connections
  if (rclcpp::ok() && grasp_action_client->wait_for_action_server(
                          rclcpp::Duration::from_seconds(10.0).to_chrono<std::chrono::seconds>()))
  {
    RCLCPP_INFO(node->get_logger(), "Found grasp action server");
  }
  else
  {
    throw std::runtime_error("Failed to find server");
  }

  if (rclcpp::ok() && target_recognition_client->wait_for_service(
                          rclcpp::Duration::from_seconds(10.0).to_chrono<std::chrono::seconds>()))
  {
    RCLCPP_INFO(node->get_logger(), "Found target detection server");
  }
  else
  {
    throw std::runtime_error("Failed to find server");
  }
}
}  // namespace pick_and_place_application
