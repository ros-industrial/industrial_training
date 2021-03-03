#include <rclcpp/rclcpp.hpp>
#include <fake_ar_publisher/msg/ar_marker.hpp>
#include <myworkcell_core/srv/localize_part.hpp>

class Localizer : public rclcpp::Node
{
public:
  Localizer() : Node("vision_node"), last_msg_{nullptr}
  {
    ar_sub_ = this->create_subscription<fake_ar_publisher::msg::ARMarker>(
        "ar_pose_marker",
        rclcpp::QoS(1),
        std::bind(&Localizer::visionCallback, this, std::placeholders::_1));

    server_ = this->create_service<myworkcell_core::srv::LocalizePart>(
        "localize_part",
        std::bind(&Localizer::localizePart, this, std::placeholders::_1, std::placeholders::_2));
  }

  void visionCallback(fake_ar_publisher::msg::ARMarker::SharedPtr msg)
  {
    last_msg_ = msg;
    /*
    RCLCPP_INFO(get_logger(), "Received pose: x=%f, y=%f, z=%f",
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z);
    */
  }

  void localizePart(myworkcell_core::srv::LocalizePart::Request::SharedPtr req,
                  myworkcell_core::srv::LocalizePart::Response::SharedPtr res)
  {
    // Read last message
    fake_ar_publisher::msg::ARMarker::SharedPtr p = last_msg_;

    if (! p)
    {
      RCLCPP_ERROR(this->get_logger(), "no data");
      res->success = false;
      return;
    }

    res->pose = p->pose.pose;
    res->success = true;
  }

  rclcpp::Subscription<fake_ar_publisher::msg::ARMarker>::SharedPtr ar_sub_;
  rclcpp::Service<myworkcell_core::srv::LocalizePart>::SharedPtr server_;
  fake_ar_publisher::msg::ARMarker::SharedPtr last_msg_;
};

int main(int argc, char* argv[])
{
  // This must be called before anything else ROS-related
  rclcpp::init(argc, argv);

  // The Localizer class provides this node's ROS interfaces
  auto node = std::make_shared<Localizer>();

  RCLCPP_INFO(node->get_logger(), "Vision node starting");

  // Don't exit the program.
  rclcpp::spin(node);
}
