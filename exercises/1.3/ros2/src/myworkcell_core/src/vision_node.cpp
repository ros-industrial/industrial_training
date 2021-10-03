/**
**  Simple ROS Node
**/
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
  // This must be called before anything else ROS-related
  rclcpp::init(argc, argv);

  // Create a ROS node
  auto node = std::make_shared<rclcpp::Node>("vision_node");

  RCLCPP_INFO(node->get_logger(), "Hello, World!");

  // Don't exit the program.
  rclcpp::spin(node);
}
