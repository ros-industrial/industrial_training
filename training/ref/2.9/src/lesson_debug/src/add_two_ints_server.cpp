/*
* Content from http://www.ros.org/wiki/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29
*
*/
 
#include "ros/ros.h"
#include "lesson_debug/AddTwoInts.h"

bool add(lesson_debug::AddTwoInts::Request  &req,
         lesson_debug::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}
