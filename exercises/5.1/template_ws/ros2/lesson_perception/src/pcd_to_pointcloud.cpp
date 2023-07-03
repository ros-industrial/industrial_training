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

  std::string filename, tf_frame, topic;
  node->declare_parameter("filename", "table.pcd");
  node->declare_parameter("frame_id", "kinect_link");
  node->declare_parameter("cloud_pcd", "/kinect/depth_registered/points");

  node->get_parameter("filename", filename);
  node->get_parameter("frame_id", tf_frame);
  node->get_parameter("cloud_pcd", topic);

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
      rclcpp::sleep_for(std::chrono::nanoseconds(rclcpp::Duration(1, 0).nanoseconds())); // 1 second
      rclcpp::spin_some(node);
    }
  }

  rclcpp::shutdown();
  return 0;
}
