#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lesson_move_group");

  // start a background "spinner", so our node can process ROS messages
  //  - this lets us know when the move is completed
  ros::AsyncSpinner spinner(1);
  spinner.start();

  move_group_interface::MoveGroup group("manipulator");

  Eigen::Affine3d pose = Eigen::Translation3d(-0.358, 0.043, 0.865)
                         * Eigen::Quaterniond(0.690, 0.108, 0.706, -0.112);
  group.setPoseTarget(pose);
  group.move();
}
