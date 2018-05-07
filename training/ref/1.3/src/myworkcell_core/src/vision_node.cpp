#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");

  ros::NodeHandle nh;

  ROS_INFO("Hello, World from a ROS Node that will become the 'vision' node in a Scan-N-Plan application!");
  ros::spin();
}

