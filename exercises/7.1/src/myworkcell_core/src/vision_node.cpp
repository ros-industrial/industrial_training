/**
**  Simple ROS2 Node; WIP
**/
#include <rclcpp/rclcpp.hpp>
#include <fake_ar_publisher_msgs/msg/ar_marker.hpp>
#include "myworkcell_core/srv/localize_part.hpp"
#include <iostream>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class Localizer : public rclcpp::Node
{
public:
    Localizer()
    : Node("vision_node")
    {    
        ar_sub_ = this->create_subscription<fake_ar_publisher_msgs::msg::ARMarker>("ar_pose_marker", 1,
        std::bind(&Localizer::visionCallback, this, std::placeholders::_1));

        server_ = this->create_service<myworkcell_core::srv::LocalizePart>("localize_part", 
        std::bind(&Localizer::localizePart, this, _1, _2, _3));
    }

    void visionCallback(const fake_ar_publisher_msgs::msg::ARMarker::SharedPtr msg) //need to sort out this shared pointer situation
    {
      last_msg_ = msg;
    }

    bool localizePart (const std::shared_ptr<rmw_request_id_t> request_header,
                      const std::shared_ptr<myworkcell_core::srv::LocalizePart::Request> req,
                      std::shared_ptr<myworkcell_core::srv::LocalizePart::Response> res) //server response & request in ROS2?
    {
      // Read last message
      fake_ar_publisher_msgs::msg::ARMarker::SharedPtr p = last_msg_;
      
      if (!p){
        std::cout <<"no data" << std::endl;
        return false;
      }

      res->pose = p->pose.pose;
      return true;
    }

    rclcpp::Subscription<fake_ar_publisher_msgs::msg::ARMarker>::SharedPtr ar_sub_;
    fake_ar_publisher_msgs::msg::ARMarker::SharedPtr last_msg_;
    rclcpp::Service<myworkcell_core::srv::LocalizePart>::SharedPtr server_;
};

int main(int argc, char* argv[])
{
    // This must be called before anything else ROS-related
    rclcpp::init(argc, argv);

    // Don't exit the program.
    rclcpp::spin(std::make_shared<Localizer>());
}
