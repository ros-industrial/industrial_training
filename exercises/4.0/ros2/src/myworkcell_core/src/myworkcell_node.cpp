#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <myworkcell_core/srv/localize_part.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

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

  void start(const std::string& base_frame)
  {
    RCLCPP_INFO(get_logger(), "Attempting to localize part");

    // Create a request for the LocalizePart service call
    auto request = std::make_shared<myworkcell_core::srv::LocalizePart::Request>();
    request->base_frame = base_frame;

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

    RCLCPP_INFO(this->get_logger(), "Part Localized:  x: %f, y: %f, z: %f",
        response->pose.position.x,
        response->pose.position.y,
        response->pose.position.z);

    geometry_msgs::msg::PoseStamped move_target;
    move_target.header.frame_id = base_frame;
    move_target.pose = response->pose;

    RCLCPP_INFO(this->get_logger(), "Creating move group interface");
    moveit::planning_interface::MoveGroupInterface move_group(
        this->shared_from_this(),
        "manipulator");

    RCLCPP_INFO(this->get_logger(), "Setting move target pose");
    move_group.setPoseTarget(move_target);

    RCLCPP_INFO(this->get_logger(), "Ready to plan and move!");
    move_group.move();
    RCLCPP_INFO(this->get_logger(), "Move completed");
  }

private:
  // Planning components
  rclcpp::Client<myworkcell_core::srv::LocalizePart>::SharedPtr vision_client_;
};

int main(int argc, char **argv)
{
  // This must be called before anything else ROS-related
  rclcpp::init(argc, argv);

  // Create the ScanNPlan node
  auto app = std::make_shared<ScanNPlan>();

  std::string base_frame;
  app->get_parameter("base_frame", base_frame);

  //Wait for the vision node to receive data
  rclcpp::sleep_for(std::chrono::seconds(2));

  // Start spinning in a background thread so MoveIt internals can execute
  std::thread worker{
    [app]()
    {
      rclcpp::spin(app);
    }
  };

  // Run our application
  app->start(base_frame);
  rclcpp::shutdown();

  //Wait for the background worker to terminate
  worker.join();

  return 0;
}
