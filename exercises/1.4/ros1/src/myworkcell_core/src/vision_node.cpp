/**
**  Simple ROS Node
**/
#include <ros/ros.h>
#include <fake_ar_publisher/ARMarker.h>

class Localizer
{
public:
    Localizer(ros::NodeHandle& nh)
    {
        ar_sub_ = nh.subscribe<fake_ar_publisher::ARMarker>("ar_pose_marker", 1,
        &Localizer::visionCallback, this);
    }

    void visionCallback(const fake_ar_publisher::ARMarkerConstPtr& msg)
    {
        last_msg_ = msg;
        ROS_INFO_STREAM(last_msg_->pose.pose);
    }

    ros::Subscriber ar_sub_;
    fake_ar_publisher::ARMarkerConstPtr last_msg_;
};

int main(int argc, char* argv[])
{
    // This must be called before anything else ROS-related
    ros::init(argc, argv, "vision_node");

    // Create a ROS node handle
    ros::NodeHandle nh;

    // The Localizer class provides this node's ROS interfaces
    Localizer localizer(nh);

    ROS_INFO("Vision node starting");

    // Don't exit the program.
    ros::spin();
}
