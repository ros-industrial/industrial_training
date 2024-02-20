#include <chrono>
#include <functional>
#include <memory>
#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <camera_calibration_parsers/parse.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::filesystem;
using namespace std::placeholders;

class CameraInfoPublisher : public rclcpp::Node
{
  public:
    CameraInfoPublisher() : Node("camera_info_publisher")
    {
        camera_ = this->declare_parameter("camera", "camera0");

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 10,
              std::bind(&CameraInfoPublisher::image_callback, this, _1));

        publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/camera_info", 10);
    }

  private:
    void image_callback(sensor_msgs::msg::Image::SharedPtr img_msg)
    {
      // start with a blank message
      sensor_msgs::msg::CameraInfo info;
      path file_path = ament_index_cpp::get_package_share_directory("cal_demo_intrinsics");
      file_path /= path("config") / path(camera_ + "_cal") / path("ost.yaml");

      std::string saved_name; // camera name in file - to be loaded

      // parse the calibration into a CameraInfo message
      if (!false /* CODE HERE! */)
      {
        RCLCPP_INFO(this->get_logger(), "Error parsing calibration");
        return;
      }

      // need to fill in timestamp and frame info
      // CODE HERE!
      // update the CameraInfo header to match the image header

      RCLCPP_INFO(this->get_logger(), "Publishing camera info");

      publisher_->publish(info);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher_;
    std::string camera_;
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraInfoPublisher>());
  rclcpp::shutdown();
  return 0;
}