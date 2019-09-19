#include <rclcpp/rclcpp.hpp>
#include "myworkcell_core/srv/localize_part.hpp"


class ScanNPlan : public rclcpp::Node
{
public:
  ScanNPlan(rclcpp::Node& node)
  : Node("scan_n_plan")
  {
    vision_client_ = this->create_client<myworkcell_core::srv::LocalizePart>("localize_part");
  }

  void start(const std::string& base_frame)
  {
    RCLCPP_INFO(this->get_logger(), "Attempting to localize part");
    // Localize the part
    myworkcell_core::srv::LocalizePart srvr; //srv changed to srvr to avoid ambiguity
    srvr.request.base_frame = base_frame;
    RCLCPP_INFO(this->get_logger(), "Requesting pose in base frame: %s", base_frame);

    if (!vision_client_->call(srv))
    {
      RCLCPP_ERROR(this->get_logger(), "Could not localize part");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "part localized: %s", srvr.response);
  }

private:
  //
  rclcpp::Client<myworkcell_core::srv::LocalizePart>::SharedPtr vision_client_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("myworkcell_node");
  RCLCPP_INFO(node->get_logger(), "ScanNPlan node has been initialized");

  std::string base_frame;
// parameter name, string object reference, default value

  ScanNPlan app();

  rclcpp::Duration(.5).sleep();  // wait for the class to initialize
  app.start(base_frame);

  rclcpp::spin();
}
