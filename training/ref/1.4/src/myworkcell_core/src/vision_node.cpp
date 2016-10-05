#include <ros/ros.h>
#include <std_msgs/String.h>
#include <myworkcell_core/LocalizePart.h>
#include <tf/transform_listener.h>
#include <myworkcell_core/ARMarker.h>

class Localizer
{
public:
  Localizer(ros::NodeHandle& nh)
  {
    ar_sub_ = nh.subscribe<myworkcell_core::ARMarker>("ar_pose_marker", 1, 
      &Localizer::visionCallback, this);

    server_ = nh.advertiseService("localize_part", &Localizer::localizePart, this);
  }

  bool localizePart(myworkcell_core::LocalizePart::Request& req,
                    myworkcell_core::LocalizePart::Response& res)
  {
    // Read last message
    myworkcell_core::ARMarkerConstPtr p = last_msg_;  
    if (!p) return false;

    res.pose = p->pose.pose;
    return true;
  }

  void visionCallback(const myworkcell_core::ARMarkerConstPtr& msg)
  {
    last_msg_ = msg;
  }

  ros::Subscriber ar_sub_;
  ros::ServiceServer server_;
  myworkcell_core::ARMarkerConstPtr last_msg_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");

  ros::NodeHandle nh;
  Localizer localizer (nh);

  ROS_INFO("Vision node starting");
  ros::spin();
}
