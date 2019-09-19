/**
**  Simple ROS2 Node; WIP
**/

#include <rclcpp/rclcpp.hpp>
#include <fake_ar_publisher_msgs/msg/ARMarker.hpp>
#include <myworkcell_core/srv/LocalizePart.hpp>

class Localizer
{
public:
    Localizer(rclcpp::Node& node)
    {
        ar_sub_ = node->create_subscription<fake_ar_publisher::msg::ARMarker>("ar_pose_marker", 1,
        &Localizer::visionCallback, this);

        server_ = node->create_service("localize_part", &Localizer::localizePart, this);
    }

    void visionCallback(const fake_ar_publisher::msg::ARMarkerConstPtr& msg)
    {
        last_msg_ = msg;
        RCLPP_INFO(node->get_logger(), last_msg_->pose.pose); //ros2 INFO_STREAM?
    }

    bool localizePart(myworkcell_core::srv::LocalizePart::Request& req,
                      myworkcell_core::srv::LocalizePart::Response& res) //server response & request in ROS2?
    {
      // Read last message
      fake_ar_publisher::msg::ARMarkerConstPtr p = last_msg_;
      if (!p) return false;

      res.pose = p->pose.pose;
      return true;
    }

    ros::Subscriber ar_sub_;
    fake_ar_publisher::msg::ARMarkerConstPtr last_msg_;
    ros::ServiceServer server_;
};

int main(int argc, char* argv[])
{
    // This must be called before anything else ROS-related
    rclcpp::init(argc, argv);

    // Create a ROS node. There are not node handles in ros 2
    auto node = rclcpp::Node::make_shared("vision_node");

    // The Localizer class provides this node's ROS interfaces
    Localizer localizer(node);

    RCLCPP_INFO(node->get_logger(),"Vision node starting");

    // Don't exit the program.
    rclcpp::spin_some(node);
}
