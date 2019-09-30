#include <rclcpp/rclcpp.hpp>
#include <fake_ar_publisher_msgs/msg/ar_marker.hpp>
#include <visualization_msgs/msg/marker.hpp>

using namespace std::chrono_literals;

static std::string& camera_frame_name()
{
  static std::string camera_frame;
  return camera_frame;
} 

// Singleton Instance of Object Position
static geometry_msgs::msg::Pose& pose()
{
  static geometry_msgs::msg::Pose pose;
  return pose;
}

// Given a marker w/ pose data, publish an RViz visualization
// You'll need to add a "Marker" visualizer in RVIZ AND define
// the "camera_frame" TF frame somewhere to see it.

class FakeARPublisher : public rclcpp::Node
{
public:
  FakeARPublisher()
  : Node("fake_ar_publisher")
  {
    ar_pub_ = this->create_publisher<fake_ar_publisher_msgs::msg::ARMarker>("ar_pose_marker", 1);
    visual_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("ar_pose_visual", 1);
    timer_ = this->create_wall_timer(100ms, std::bind(&FakeARPublisher::pubCallback, this));
    
  }

private:
  void pubVisualMarker(const fake_ar_publisher_msgs::msg::ARMarker& m)
  {
    const double width = 0.08;
    const double thickness = 0.005;
    
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = m.header.frame_id;
    marker.header.stamp = this->now();
    marker.ns = "ar_marker_visual";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = m.pose.pose;
    marker.pose.position.z -= thickness / 2.0;
    marker.scale.x = width;
    marker.scale.y = width;
    marker.scale.z = thickness;
    marker.color.a = 1.0;
    marker.color.b = 1.0;
    
    visual_pub_->publish(marker);
  }


  void pubCallback()
  {
    geometry_msgs::msg::Pose p = pose();
    fake_ar_publisher_msgs::msg::ARMarker m;
    m.header.frame_id = camera_frame_name();
    m.header.stamp = this->now();
    m.pose.pose = p;

    ar_pub_->publish(m);
    
    pubVisualMarker(m); // visualize the marker
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<fake_ar_publisher_msgs::msg::ARMarker>::SharedPtr ar_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr visual_pub_;
};

int main(int argc, char **argv)
{
  // Set up ROS.
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FakeARPublisher>();
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node);

  node->declare_parameter("x");
  node->declare_parameter("y");
  node->declare_parameter("z");
  node->declare_parameter("camera_frame");

  // init pose
  pose().orientation.w = 1.0; // facing straight up
  node->get_parameter("x", pose().position.x);
  node->get_parameter("y", pose().position.y);
  node->get_parameter("z", pose().position.z);
  node->get_parameter("camera_frame", camera_frame_name());

  RCLCPP_INFO(node->get_logger(), "Starting simulated ARMarker publisher");  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
