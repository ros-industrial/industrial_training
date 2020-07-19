#include <rclcpp/rclcpp.hpp>
#include <myworkcell_core/srv/localize_part.hpp>

class ScanNPlan : public rclcpp::Node
{
public:
  ScanNPlan() : Node("scan_n_plan")
  {
    vision_client_ = this->create_client<myworkcell_core::srv::LocalizePart>("localize_part");
  }

  void start()
  {
    RCLCPP_INFO(get_logger(), "Attempting to localize part");

    // Create a request for the LocalizePart service call
    auto request = std::make_shared<myworkcell_core::srv::LocalizePart::Request>();

    auto future = vision_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) != rclcpp::executor::FutureReturnCode::SUCCESS)
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
  app->start();

  rclcpp::shutdown();
  return 0;
}
