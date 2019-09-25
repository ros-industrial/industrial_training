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
    // Need to wait until vision node has data
    rclcpp::Rate rate(std::chrono::duration<int, std::milli>(1000));
    rate.sleep();

    RCLCPP_INFO(this->get_logger(), "Attempting to localize part in frame: %s", base_frame.c_str());
    
    while (!vision_client_->wait_for_service(std::chrono::duration<int, std::milli>(500))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
        return;
      } 
      RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
    }

    // Localize the part
    auto request = std::make_shared<myworkcell_core::srv::LocalizePart::Request>();
    request->base_frame = base_frame;
    
    using ServiceResponseFuture =
      rclcpp::Client<myworkcell_core::srv::LocalizePart>::SharedFuture; //holds results of async call

    auto response_received_callback = [this](ServiceResponseFuture future) { // we need to have less autos; future is empty
        auto result = future.get();
        RCLCPP_INFO(this->get_logger(), "Part Localized:  w: %f, x: %f, y: %f, z: %f",
                                      result->pose.orientation.w, 
                                      result->pose.position.x, 
                                      result->pose.position.y, 
                                      result->pose.position.z);
        rclcpp::shutdown();
      };    

    auto future = vision_client_->async_send_request(request, response_received_callback);
    // if (future.wait_for(std::chrono::duration<int, std::milli>(1000)) ==
    //   std::future_status::timeout)
    // {
    //   RCLCPP_ERROR(this->get_logger(), "Could not localize part");
    //   return;
    // }

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
  rclcpp::spin(app);
  rclcpp::shutdown();
  return 0;
}
