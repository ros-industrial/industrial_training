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

  Eigen::Affine3d approach = Eigen::Translation3d(0.309, 0.369, 0.266) *
                             Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
  group.setPoseTarget(approach);
  group.move();
  sleep(1);

  Eigen::Affine3d pick = approach.translate(0.2*Eigen::Vector3d::UnitZ());
  group.setPoseTarget(pick);
  group.move();
  sleep(1);

  Eigen::Affine3d retreat = pick.translate(-0.1*Eigen::Vector3d::UnitZ());
  group.setPoseTarget(retreat);
  group.move();
  sleep(1);

  std::vector<double> inspectPos;
  inspectPos.push_back(-0.85); inspectPos.push_back(-1.24); inspectPos.push_back(1.885);
  inspectPos.push_back(-0.58); inspectPos.push_back(2.38); inspectPos.push_back(1.34);
  group.setJointValueTarget(inspectPos);
  group.move();
  sleep(1);

  group.setNamedTarget("home");
  group.move();
}
