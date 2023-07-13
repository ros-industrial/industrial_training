#include <chrono>
#include <cinttypes>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "myworkcell_core/srv/localize_part.hpp"

// The ScanNPlan inherits from node, allowing it to act as a Node to do things like create clients
class ScanNPlan : public rclcpp::Node
{
public:
  explicit ScanNPlan()
  : Node("scan_n_plan")
  {
    vision_client_ = this->create_client<myworkcell_core::srv::LocalizePart>("localize_part");
  }

  void start(const std::string& base_frame)
  {
    using namespace std::chrono_literals;

    // Need to wait until vision node has data
    rclcpp::sleep_for(1s);

    RCLCPP_INFO(this->get_logger(), "Attempting to localize part in frame: %s", base_frame.c_str());

    // The vision client needs to wait until the service appears
    while (!vision_client_->wait_for_service(500ms)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
    }

    // Create a request for the LocalizePart service call
    auto request = std::make_shared<myworkcell_core::srv::LocalizePart::Request>();
    // The base_frame that is passed in is used to fill the request
    request->base_frame = base_frame;

    auto future = vision_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to receive LocalizePart service response");
      return;
    }

    auto result = future.get();
    if (! result->success)
    {
      RCLCPP_ERROR(this->get_logger(), "LocalizePart service failed");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Part Localized:  w: %f, x: %f, y: %f, z: %f",
        result->pose.orientation.w,
        result->pose.position.x,
        result->pose.position.y,
        result->pose.position.z);
  }

private:
  rclcpp::Client<myworkcell_core::srv::LocalizePart>::SharedPtr vision_client_;

};

int main(int argc, char **argv)
{
  // This must be called before anything else ROS-related
  rclcpp::init(argc, argv);

  // Create the ScanNPlan object and node
  auto app = std::make_shared<ScanNPlan>();

  // String to store the base_frame parameter after getting it from the Node's parameter client
  std::string base_frame;
  app->declare_parameter("base_frame", "");
  app->get_parameter("base_frame", base_frame);

  if (base_frame.empty())
  {
    RCLCPP_ERROR(app->get_logger(), "No 'base_frame' parameter provided");
    return -1;
  }

  RCLCPP_INFO(app->get_logger(), "ScanNPlan node has been initialized");

  // Call the vision client's LocalizePart service using base_frame as a parameter
  app->start(base_frame);

  rclcpp::shutdown();
  return 0;
}
