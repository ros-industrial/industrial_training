#ifndef PICK_AND_PLACE_H_
#define PICK_AND_PLACE_H_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/planning_scene.hpp>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>

#include <pick_and_place_msgs/srv/get_target_pose.hpp>
#include <pick_and_place_msgs/action/execute_grasp_move.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/create_timer_ros.h>

#include <pick_and_place_application/pick_and_place_utilities.h>

#include <geometric_shapes/shape_operations.h>

// =============================== aliases ===============================
using ExecuteGraspAction = pick_and_place_msgs::action::ExecuteGraspMove;
using ExecuteGraspActionClient = rclcpp_action::Client<ExecuteGraspAction>;

namespace pick_and_place_application
{
class PickAndPlaceApp
{
public:
  // =============================== constructor =====================================
  PickAndPlaceApp(rclcpp::Node::SharedPtr node) : node(node), transform_buffer(node->get_clock())
  {
    tf2_ros::CreateTimerInterface::SharedPtr timer_intf =
        std::make_shared<tf2_ros::CreateTimerROS>(node->get_node_base_interface(), node->get_node_timers_interface());
    transform_buffer.setCreateTimerInterface(timer_intf);
    // transform_buffer.setUsingDedicatedThread(false);
    transform_listener = std::make_shared<tf2_ros::TransformListener>(transform_buffer);
  }

  // =============================== public members =====================================
  PickAndPlaceConfig cfg;
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_publisher;
  rclcpp::Client<pick_and_place_msgs::srv::GetTargetPose>::SharedPtr target_recognition_client;

  ExecuteGraspActionClient::SharedPtr grasp_action_client;
  moveit_cpp::MoveItCppPtr moveit_cpp;

  tf2_ros::Buffer transform_buffer;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener;

  // =============================== Task Functions ===============================
  void initialize();

  void moveToWaitPosition();

  void actuateGripper(bool do_grasp);

  void setAttachedObject(bool attach, const geometry_msgs::msg::Pose& pose, moveit_msgs::msg::RobotState& robot_state);

  void resetWorld(bool do_perception = true);

  geometry_msgs::msg::Pose detectBox();

  std::vector<geometry_msgs::msg::Pose> computePickToolPoses(geometry_msgs::msg::Pose& box_pose);

  std::vector<geometry_msgs::msg::Pose> computePlaceToolPoses();

  void doBoxPickup(std::vector<geometry_msgs::msg::Pose>& pick_poses, const geometry_msgs::msg::Pose& box_pose);

  void doBoxPlace(std::vector<geometry_msgs::msg::Pose>& place_poses, const geometry_msgs::msg::Pose& box_pose);

  bool doMotionPlanning(const geometry_msgs::msg::Pose& pose_target,
                        const moveit_msgs::msg::RobotState& start_robot_state,
                        moveit_cpp::PlanningComponent::PlanSolution& plan);

  void showBox(bool show = true)
  {
    // updating marker action
    cfg.MARKER_MESSAGE.action = show ? visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;

    // publish messages
    marker_publisher->publish(cfg.MARKER_MESSAGE);
  }
};
}  // namespace pick_and_place_application

#endif /* PICK_AND_PLACE_H_ */
