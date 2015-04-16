#include <plan_and_run/demo_application.h>

namespace plan_and_run
{

void DemoApplication::runPath(const DescartesTrajectory& path)
{

  // creating move group to move the arm in free space
  move_group_interface::MoveGroup move_group(config_.group_name);
  move_group.setPlannerId(PLANNER_ID);

  // creating goal joint pose to start of the path
  std::vector<double> seed_pose(robot_model_ptr_->getDOF()),start_pose;
  path[0]->getNominalJointPose(seed_pose,*robot_model_ptr_,start_pose);

  // moving arm to joint goal
  move_group.setJointValueTarget(start_pose);
  move_group.setPlanningTime(10.0f);
  moveit_msgs::MoveItErrorCodes result = move_group.move();
  if(result.val != result.SUCCESS)
  {
    ROS_ERROR_STREAM("Move to start joint pose failed");
    exit(-1);
  }

  // sending path to robot
  moveit_msgs::RobotTrajectory moveit_traj;
  fromDescartesToMoveitTrajectory(path,moveit_traj.joint_trajectory);
  moveit_msgs::ExecuteKnownTrajectory srv;
  srv.request.trajectory = moveit_traj;
  srv.request.wait_for_execution = true;

  ROS_INFO_STREAM("Robot path sent for execution");
  if(moveit_run_path_client_.call(srv))
  {
    ROS_INFO_STREAM("Robot path execution completed");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to run robot path with error "<<srv.response.error_code.val);
    exit(-1);
  }

}

}
