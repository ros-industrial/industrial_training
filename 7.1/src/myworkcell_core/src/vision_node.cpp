/**
**  Simple ROS Node
**/
#include <rclcpp/rclcpp.hpp>
#include <fake_ar_publisher/ARMarker.h>
#include <myworkcell_core/LocalizePart.h>

class Localizer
{
public:
    Localizer(rclcpp::Node& nh)
    {
        ar_sub_ = nh.subscribe<fake_ar_publisher::ARMarker>("ar_pose_marker", 1,
        &Localizer::visionCallback, this);

        server_ = nh.advertiseService("localize_part", &Localizer::localizePart, this);
    }

    void visionCallback(const fake_ar_publisher::ARMarkerConstPtr& msg)
    {
        last_msg_ = msg;
        //ROS_INFO_STREAM(last_msg_->pose.pose);
    }

    bool localizePart(myworkcell_core::LocalizePart::Request& req,
                      myworkcell_core::LocalizePart::Response& res)
    {
      // Read last message
      fake_ar_publisher::ARMarkerConstPtr p = last_msg_;
      if (!p) return false;

      res.pose = p->pose.pose;
      return true;
    }

    ros::Subscriber ar_sub_;
    fake_ar_publisher::ARMarkerConstPtr last_msg_;
    ros::ServiceServer server_;
};

int main(int argc, char* argv[])
{
    // This must be called before anything else ROS-related
    rclcpp::init(argc, argv);

    // Create a ROS node handle
    auto node = rclcpp::Node::make_shared("vision_node");

    // The Localizer class provides this node's ROS interfaces
    Localizer localizer(node);

    RCLCPP_INFO("Vision node starting");

    // Don't exit the program.
    rclcpp::spin();
}
