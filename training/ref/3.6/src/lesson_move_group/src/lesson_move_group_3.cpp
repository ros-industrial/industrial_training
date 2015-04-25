#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lesson_move_group");

  // start a background "spinner", so our node can process ROS messages
  //  - this lets us know when the move is completed
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::map<std::string, double> joints;
  joints["joint_1"] = -0.40;
  joints["joint_2"] =  0.27;
  joints["joint_3"] =  0.48;
  joints["joint_4"] =  0.07;
  joints["joint_5"] =  0.75;
  joints["joint_6"] = -0.19;

  moveit::planning_interface::MoveGroup group("manipulator");
  group.setJointValueTarget(joints);
  group.move();
}
