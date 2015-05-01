#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lesson_move_group");

  // start a background "spinner", so our node can process ROS messages
  //  - this lets us know when the move is completed
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroup group("manipulator");

  Eigen::Affine3d pose = Eigen::Translation3d(0.306, 0.365, 0.140)
                         * Eigen::Quaterniond(-0.000, 1.000, 0.009, 0.009);
  group.setPoseTarget(pose);
  group.move();
}
