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

  Eigen::Affine3d pose = Eigen::Translation3d(0.028, 0.793, 0.390)
                         * Eigen::Quaterniond(-0.014, 0.733, 0.680, -0.010);
  group.setPoseTarget(pose);
  group.move();
}
