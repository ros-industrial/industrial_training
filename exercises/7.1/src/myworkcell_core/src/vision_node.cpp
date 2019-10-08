#include <rclcpp/rclcpp.hpp>
#include <fake_ar_publisher/msg/ar_marker.hpp>
#include <myworkcell_core/srv/localize_part.hpp>

class Localizer : public rclcpp::Node
{
public:
    Localizer()
    : Node("vision_node")
    {
        using namespace std::placeholders;

        ar_sub_ = this->create_subscription<fake_ar_publisher::msg::ARMarker>("ar_pose_marker", rclcpp::QoS(1),
          std::bind(&Localizer::visionCallback, this, std::placeholders::_1));

        server_ = this->create_service<myworkcell_core::srv::LocalizePart>("localize_part",
          std::bind(&Localizer::localizePart, this, _1, _2));
    }

    void visionCallback(fake_ar_publisher::msg::ARMarker::SharedPtr msg)
    {
      last_msg_ = msg;
    }

    void localizePart(std::shared_ptr<myworkcell_core::srv::LocalizePart::Request> req,
                      std::shared_ptr<myworkcell_core::srv::LocalizePart::Response> res)
    {
      // Read last message
      fake_ar_publisher::msg::ARMarker::SharedPtr p = last_msg_;

      if (!p){
        RCLCPP_ERROR(this->get_logger(), "no data");
        res->success = false;
        return;
      }

      res->success = true;
      res->pose = p->pose.pose;
    }

    rclcpp::Subscription<fake_ar_publisher::msg::ARMarker>::SharedPtr ar_sub_;
    rclcpp::Service<myworkcell_core::srv::LocalizePart>::SharedPtr server_;
    fake_ar_publisher::msg::ARMarker::SharedPtr last_msg_;
};

int main(int argc, char* argv[])
{
    // This must be called before anything else ROS-related
    rclcpp::init(argc, argv);

    // Don't exit the program.
    rclcpp::spin(std::make_shared<Localizer>());
}
