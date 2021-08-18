/*
 * initialize.cpp
 *
 *  Created on: Aug 13, 2021
 *      Author: jnicho
 */

#include <collision_avoidance_pick_and_place/pick_and_place.h>

/* INITIALIZE
  Goal:
    - Loads node parameters and initializes ROS2 interfaces (actions, subscribers, publishers, services)
*/

namespace collision_avoidance_pick_and_place
{
  bool PickAndPlaceApp::initialize()
  {
    // reading parameters
    if(this->cfg.init(node))
    {
      RCLCPP_INFO(node->get_logger(), "Parameters successfully loaded");
    }
    else
    {
      RCLCPP_ERROR(node->get_logger(), "Parameters not found");
      return false;
    }

    // marker publisher
    marker_publisher = node->create_publisher<visualization_msgs::msg::Marker>(cfg.MARKER_TOPIC,1);

    // planning scene publisher
    planning_scene_publisher = node->create_publisher<moveit_msgs::msg::PlanningScene>(cfg.PLANNING_SCENE_TOPIC,1);

    // target recognition client (perception)
    target_recognition_client = node->create_client<pick_and_place_msgs::srv::GetTargetPose>(cfg.TARGET_RECOGNITION_SERVICE);

    // grasp action client (vacuum gripper)
    grasp_action_client = rclcpp_action::create_client<ExecuteGraspAction>(node, cfg.GRASP_ACTION_NAME);

    // instantiate the moveit cpp object for motion planing
    moveit_cpp = std::make_shared<moveit_cpp::MoveItCpp>(node);

    // waiting to establish connections
    /*
    while(ros::ok() &&
        !application.grasp_action_client_ptr->waitForServer(ros::Duration(2.0f)))
    {
      ROS_INFO_STREAM("Waiting for grasp action servers");
    }

    if(ros::ok() && !application.target_recognition_client.waitForExistence(ros::Duration(2.0f)))
    {
      ROS_INFO_STREAM("Waiting for service'"<<application.cfg.TARGET_RECOGNITION_SERVICE<<"'");
    }*/

    return true;

  }
}



