#include <rclcpp/rclcpp.hpp>
#include <fake_ar_publisher_msgs/msg/ar_marker.hpp>
#include <visualization_msgs/msg/marker.hpp>


rclcpp::Publisher ar_pub;
rclcpp::Publisher visual_pub;

static std::string& camera_frame_name()
{
  static std::string camera_frame;
  return camera_frame;
} 

// Singleton Instance of Object Position
static geometry_msgs::Pose& pose()
{
  static geometry_msgs::Pose pose;
  return pose;
}

// Given a marker w/ pose data, publish an RViz visualization
// You'll need to add a "Marker" visualizer in RVIZ AND define
// the "camera_frame" TF frame somewhere to see it.
static void pubVisualMarker(const fake_ar_publisher_msgs::ARMarker& m)
{
  const double width = 0.08;
  const double thickness = 0.005;
  
  visualization_msgs::Marker marker;
  marker.header.frame_id = m.header.frame_id;
  marker.header.stamp = rclcpp::Time::now();
  marker.ns = "ar_marker_visual";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = m.pose.pose;
  marker.pose.position.z -= thickness / 2.0;
  marker.scale.x = width;
  marker.scale.y = width;
  marker.scale.z = thickness;
  marker.color.a = 1.0;
  marker.color.b = 1.0;
  
  visual_pub.publish(marker);
}

void pubCallback(const rclcpp::TimerEvent&)
{
  geometry_msgs::Pose p = pose();
  fake_ar_publisher::ARMarker m;
  m.header.frame_id = camera_frame_name();
  m.header.stamp = rclcpp::Time::now();
  m.pose.pose = p;

  ar_pub.publish(m);
  
  pubVisualMarker(m); // visualize the marker
}

int main(int argc, char **argv)
{
  // Set up ROS.
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("fake_ar_publisher")
  auto pnh = rclcpp::Node::make_shared("~");
  ar_pub = nh.advertise<fake_ar_publisher_msgs::ARMarker>("ar_pose_marker", 1);
  visual_pub = nh.advertise<visualization_msgs::Marker>("ar_pose_visual", 1);

  // init pose
  pose().orientation.w = 1.0; // facing straight up
  pnh.param<double>("x_pos", pose().position.x, -0.6);
  pnh.param<double>("y_pos", pose().position.y, 0.2);
  pnh.param<double>("z_pos", pose().position.z, 0.5);
  
  pnh.param<std::string>("camera_frame", camera_frame_name(), "camera_frame");

  RCLCPP_INFO("Starting simulated ARMarker publisher");  
  rclcpp::Timer t = node.createTimer(rclcpp::Duration(0.1), pubCallback, false, true);
  rclcpp::spin();
}
