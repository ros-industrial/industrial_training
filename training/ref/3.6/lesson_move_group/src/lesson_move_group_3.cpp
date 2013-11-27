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
  joints["joint_s"] =  0.00;
  joints["joint_l"] = -0.48;
  joints["joint_e"] = -0.28;
  joints["joint_u"] =  1.54;
  joints["joint_r"] = -0.15;
  joints["joint_b"] =  1.12;
  joints["joint_t"] =  0.00;

  move_group_interface::MoveGroup group("manipulator");
  group.setJointValueTarget(joints);
  group.move();
}
