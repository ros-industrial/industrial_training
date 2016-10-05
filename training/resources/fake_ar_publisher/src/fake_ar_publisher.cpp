#include <ros/ros.h>
#include <fake_ar_publisher/ARMarker.h>

ros::Publisher ar_pub;

geometry_msgs::Pose& pose()
{
  static geometry_msgs::Pose pose;
  return pose;
}

void pubCallback(const ros::TimerEvent&)
{
  geometry_msgs::Pose p = pose();
  fake_ar_publisher::ARMarker m;
  m.header.frame_id = "base_link";
  m.header.stamp = ros::Time::now();
  m.pose.pose = p;

  ar_pub.publish(m);
}

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "fake_ar_publisher");
  ros::NodeHandle nh, pnh ("~");
  ar_pub = nh.advertise<fake_ar_publisher::ARMarker>("ar_pose_marker", 1);

  // init pose
  pose().orientation.w = 1.0; // facing straight up
  pnh.param<double>("x_pos", pose().position.x, 0.5);
  pnh.param<double>("y_pos", pose().position.y, 0.0);
  pnh.param<double>("z_pos", pose().position.z, 0.0);

  ros::Timer t = nh.createTimer(ros::Duration(0.1), pubCallback, false, true);
  ROS_INFO("Publishing fake ar marker data!");
  ros::spin();
}
