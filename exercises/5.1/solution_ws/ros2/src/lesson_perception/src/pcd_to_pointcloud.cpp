#include <chrono> // std::chrono::nanoseconds
#include <memory> // std::make_shared()
#include <string> // std::string

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("pcd_to_pointcloud");

  rclcpp::Parameter filename_param, tf_frame_param, topic_param;
  node->declare_parameter("filename");
  node->declare_parameter("frame_id");
  node->declare_parameter("cloud_pcd");

  node->get_parameter_or("filename", filename_param, rclcpp::Parameter("", "table.pcd"));
  node->get_parameter_or("frame_id", tf_frame_param, rclcpp::Parameter("", "kinect_link"));
  node->get_parameter_or("cloud_pcd", topic_param, rclcpp::Parameter("", "/kinect/depth_registered/points"));

  std::string filename = filename_param.as_string();
  std::string tf_frame = tf_frame_param.as_string();
  std::string topic = topic_param.as_string();

  pcl::PointCloud<pcl::PointXYZRGB> cloud;

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(filename.c_str(), cloud) < 0)
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Could not open file `" << filename << "`");
  }
  else
  {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.frame_id = tf_frame;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher =
        node->create_publisher<sensor_msgs::msg::PointCloud2>(topic, 1);

    while (rclcpp::ok())
    {
      publisher->publish(cloud_msg);
      rclcpp::sleep_for(std::chrono::nanoseconds(rclcpp::Duration(5, 0).nanoseconds())); // 5 seconds
      rclcpp::spin_some(node);
    }
  }

  rclcpp::shutdown();
  return 0;
}
