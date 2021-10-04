#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <myworkcell_core/srv/localize_part.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

class ScanNPlan : public rclcpp::Node
{
public:
  ScanNPlan() : Node("scan_n_plan", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    if (! this->has_parameter("base_frame"))
    {
      this->declare_parameter("base_frame", "world");
    }

    vision_client_ = this->create_client<myworkcell_core::srv::LocalizePart>("localize_part");
  }

  // MoveIt setup
  void setup()
  {
    // Instantiate moveit_cpp
    moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(this->shared_from_this());

    // Planning component associated with a single motion group
    planning_component_ = std::make_shared<moveit_cpp::PlanningComponent>("manipulator", moveit_cpp_);

    // Parameters set on this node
    plan_parameters_.load(this->shared_from_this());
  }

  void start(const std::string& base_frame)
  {
    RCLCPP_INFO(get_logger(), "Attempting to localize part");

    // Wait for service to be available
    if (!vision_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "Unable to find localize_part service. Start vision_node first.");
      return;
    }

    // Create a request for the LocalizePart service call
    auto request = std::make_shared<myworkcell_core::srv::LocalizePart::Request>();
    request->base_frame = base_frame;
    RCLCPP_INFO_STREAM(get_logger(), "Requesting pose in base frame: " << base_frame);

    auto future = vision_client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(3)) == std::future_status::timeout)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to receive LocalizePart service response");
      return;
    }

    auto response = future.get();
    if (! response->success)
    {
      RCLCPP_ERROR(this->get_logger(), "LocalizePart service failed");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Part Localized: x: %f, y: %f, z: %f",
        response->pose.position.x,
        response->pose.position.y,
        response->pose.position.z);

    geometry_msgs::msg::PoseStamped move_target;
    move_target.header.frame_id = base_frame;
    move_target.pose = response->pose;

    // getting current state of robot from environment
    if (!moveit_cpp_->getPlanningSceneMonitor()->requestPlanningSceneState())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to get planning scene");
      return;
    }
    moveit::core::RobotStatePtr start_robot_state = moveit_cpp_->getCurrentState(2.0);

    // Set motion goal of end effector link
    std::string ee_link = moveit_cpp_->getRobotModel()->getJointModelGroup(
        planning_component_->getPlanningGroupName())->getLinkModelNames().back();

    planning_component_->setStartState(*start_robot_state);
    planning_component_->setGoal(move_target, ee_link);

    // Now we can plan!
    moveit_cpp::PlanningComponent::PlanSolution plan_solution = planning_component_->plan(plan_parameters_);
    if (!plan_solution)
    {
      RCLCPP_ERROR(this->get_logger(),"Failed to plan");
      return;
    }

    // If planning succeeded, execute the returned trajectory
    bool success = moveit_cpp_->execute("manipulator", plan_solution.trajectory, true);
    if (!success)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to execute trajectory");
      return;
    }
  }

private:
  // Planning components
  moveit_cpp::MoveItCppPtr moveit_cpp_;
  moveit_cpp::PlanningComponentPtr planning_component_;
  moveit_cpp::PlanningComponent::PlanRequestParameters plan_parameters_;

  // perception interface
  rclcpp::Client<myworkcell_core::srv::LocalizePart>::SharedPtr vision_client_;
};

int main(int argc, char **argv)
{
  // This must be called before anything else ROS-related
  rclcpp::init(argc, argv);

  // Create the ScanNPlan node
  auto app = std::make_shared<ScanNPlan>();

  std::string base_frame = app->get_parameter("base_frame").as_string();

  //Wait for the vision node to receive data
  rclcpp::sleep_for(std::chrono::seconds(2));

  // Start spinning in a background thread so MoveIt internals can execute
  std::thread worker{ [app]() { rclcpp::spin(app); } };

  // Perform MoveIt initialization
  app->setup();

  app->start(base_frame);
  rclcpp::shutdown();

  //Wait for the background worker to terminate
  worker.join();

  return 0;
}
