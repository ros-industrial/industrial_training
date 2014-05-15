#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lesson_moveit");
  moveit::planning_interface::MoveGroup group("manipulator");

  // start a background "spinner", so our node can process ROS messages
  //  - this lets us know when the move is completed
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // INSERT MOTION COMMANDS HERE

  group.setNamedTarget("home");
  group.move();

  Eigen::Affine3d approach = Eigen::Translation3d(1.0, 0, 0.6) *
                             Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY());
  group.setPoseTarget(approach);
  group.move();

  Eigen::Affine3d pick = approach.translate(0.05*Eigen::Vector3d::UnitX());
  group.setPoseTarget(pick);
  group.move();

  Eigen::Affine3d retreat = pick.translate(-0.1*Eigen::Vector3d::UnitX());
  group.setPoseTarget(retreat);
  group.move();

  std::vector<double> inspectPos;
  inspectPos.push_back(-0.35); inspectPos.push_back(0.45); inspectPos.push_back(0.57);
  inspectPos.push_back(-0.64); inspectPos.push_back(-1.03); inspectPos.push_back(0.29);
  group.setJointValueTarget(inspectPos);
  group.move();

  group.setNamedTarget("home");
  group.move();
}
