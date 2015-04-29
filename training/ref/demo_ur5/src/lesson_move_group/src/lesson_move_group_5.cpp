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

  // create a new pose that may be out-of-reach
  geometry_msgs::Pose pose = group.getRandomPose().pose;
  //pose.position.z += 0.2;
  group.setPoseTarget(pose);

  // check to see if plan successful
  move_group_interface::MoveGroup::Plan plan;
  if (!group.plan(plan))
  {
    ROS_FATAL("Unable to create motion plan.  Aborting.");
    exit(-1);
  }

  // do non-blocking move request
  group.asyncExecute(plan);

  // cancel motion after fixed time
  sleep(1.0);
  group.stop();
  sleep(1.0);  // wait for stop command to be received
}
