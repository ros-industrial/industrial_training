#include <ros/ros.h>
#include <std_msgs/String.h>
#include <myworkcell_core/LocalizePart.h>
#include <tf/transform_listener.h>
#include <fake_ar_publisher/ARMarker.h>
#include <string>

class Localizer
{
public:
  Localizer(ros::NodeHandle& nh_in)
  {
    nh = nh_in;
    ar_sub_ = nh.subscribe<fake_ar_publisher::ARMarker>("ar_pose_marker", 1,
      &Localizer::visionCallback, this);

    server_ = nh.advertiseService("localize_part", &Localizer::localizePart, this);
  }

  bool localizePart(myworkcell_core::LocalizePart::Request& req,
                    myworkcell_core::LocalizePart::Response& res)
  {
    // Read last message
    fake_ar_publisher::ARMarkerConstPtr p = last_msg_;
    if (!p) return false;

    // Use TF to look up transform between request base frame and the camera
    tf::StampedTransform world_to_cam;
    listener_.lookupTransform(req.base_frame, p->header.frame_id, ros::Time(0), world_to_cam);

    // Use the AR pose data to a transform
    tf::Transform cam_to_target;
    tf::poseMsgToTF(p->pose.pose, cam_to_target);

    // Compute the transform world -> target
    tf::Transform world_to_target = world_to_cam * cam_to_target;

    tf::poseTFToMsg(world_to_target, res.pose);
    ROS_WARN_STREAM("TF: " << res.pose);
    ROS_ERROR_STREAM("AR: " << *p);
    return true;
  }

  void visionCallback(const fake_ar_publisher::ARMarkerConstPtr& msg)
  {
    last_msg_ = msg;
  }

  tf::TransformListener listener_;
  ros::Subscriber ar_sub_;
  ros::ServiceServer server_;
  fake_ar_publisher::ARMarkerConstPtr last_msg_;
  ros::NodeHandle nh;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");

  ros::NodeHandle nh;
  Localizer localizer (nh);

  ROS_INFO("Vision node starting");
  ros::spin();
}
