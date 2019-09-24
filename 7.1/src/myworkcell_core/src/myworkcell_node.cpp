#include <chrono>
#include <cinttypes>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "myworkcell_core/srv/localize_part.hpp"

//TODO: we are somehow blocking on this node, preventing successful run
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

    RCLCPP_INFO(this->get_logger(), base_frame);  //This cannot represent strings correctly unless given as variable (no %s)
    
    auto result_future = vision_client_->async_send_request(request);


    RCLCPP_INFO(this->get_logger(), "part g:");
    // if (result_future.wait_for(std::chrono::duration<int, std::milli>(5000)) ==
    //   std::future_status::timeout)
    // {
    //   RCLCPP_ERROR(this->get_logger(), "Could not localize part");
    //   return;
    // }
        RCLCPP_INFO(this->get_logger(), "part localized:");
    auto result = result_future.get();
        RCLCPP_INFO(this->get_logger(), "part jh:");

  }

private:
  //
  rclcpp::Client<myworkcell_core::srv::LocalizePart>::SharedPtr vision_client_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  //auto node = rclcpp::Node::make_shared("myworkcell_node");
  auto app = std::make_shared<ScanNPlan>();

  std::string base_frame;
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(app);
  RCLCPP_INFO(app->get_logger(), "ScanNPlan node has been initialized");

  app->declare_parameter("base_frame");
  app->get_parameter("base_frame", base_frame);

// parameter name, string object reference, default value


  app->start(base_frame);
  RCLCPP_INFO(app->get_logger(), "???");
  rclcpp::spin(app);
  rclcpp::shutdown();
  return 0;
}
