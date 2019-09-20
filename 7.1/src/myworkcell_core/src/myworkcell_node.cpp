#include <chrono>
#include <cinttypes>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "myworkcell_core/srv/localize_part.hpp"


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
    RCLCPP_INFO(this->get_logger(), "Attempting to localize part");

    while (!vision_client_->wait_for_service(std::chrono::duration<int, std::milli>(500))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
    }

    // Localize the part
    auto request = std::make_shared<myworkcell_core::srv::LocalizePart::Request>(); //srv changed to srvr to avoid ambiguity
    request->base_frame = base_frame;

    RCLCPP_INFO(this->get_logger(), "Requesting pose in base frame: %s", base_frame);
    auto result_future = vision_client_->async_send_request(request);

    if (result_future.wait_for(std::chrono::duration<int, std::milli>(500)) ==
      std::future_status::timeout)
    {
      RCLCPP_ERROR(this->get_logger(), "Could not localize part");
      return;
    }
    auto result = result_future.get();
    RCLCPP_INFO(this->get_logger(), "part localized: %s", result->pose);
  }

private:
  //
  rclcpp::Client<myworkcell_core::srv::LocalizePart>::SharedPtr vision_client_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("myworkcell_node");
  // auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node);
  RCLCPP_INFO(node->get_logger(), "ScanNPlan node has been initialized");

  // node->declare_parameter("base_frame");


  //std::string base_frame;
// parameter name, string object reference, default value

  auto app = std::make_shared<ScanNPlan>();
  app->start("world");

  rclcpp::spin(app);
  rclcpp::shutdown();
  return 0;
}
