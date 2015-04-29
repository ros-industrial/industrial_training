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
  joints["shoulder_pan_joint"] = -0.40;
  joints["shoulder_lift_joint"] =  -0.55;
  joints["elbow_joint"] =  0.48;
  joints["wrist_1_joint"] =  0.07;
  joints["wrist_2_joint"] =  0.75;
  joints["wrist_3_joint"] = -0.19;

  moveit::planning_interface::MoveGroup group("manipulator");
  group.setJointValueTarget(joints);
  group.move();
}
