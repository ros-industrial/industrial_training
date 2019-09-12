#include <rclcpp/rclcpp.hpp>
#include <myworkcell_core/LocalizePart.h>

class ScanNPlan
{
public:
  ScanNPlan(rclcpp::NodeHandle& nh)
  {
    vision_client_ = nh.serviceClient<myworkcell_core::LocalizePart>("localize_part");
  }

  void start(const std::string& base_frame)
  {
    RCLCPP_INFO("Attempting to localize part");
    // Localize the part
    myworkcell_core::LocalizePart srv;
    srv.request.base_frame = base_frame;
    RCLCPP_INFO("Requesting pose in base frame: %s", base_frame);

    if (!vision_client_.call(srv))
    {
      RCLCPP_ERROR("Could not localize part");
      return;
    }
    RCLCPP_INFO("part localized: %s", srv.response);
  }

private:
  // Planning components
  ros::ServiceClient vision_client_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("myworkcell_node");
  auto private_node = rclcpp::Node::make_shared("~");

  RCLCPP_INFO("ScanNPlan node has been initialized");

  std::string base_frame;
  private_node_handle.param<std::string>("base_frame", base_frame, "world"); // parameter name, string object reference, default value

  ScanNPlan app(nh);

  rclcpp::Duration(.5).sleep();  // wait for the class to initialize
  app.start(base_frame);

  rclcpp::spin();
}
