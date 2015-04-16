#include <plan_and_run/demo_application.h>

namespace plan_and_run
{

void DemoApplication::planPath(DescartesTrajectory& input_traj,DescartesTrajectory& output_path)
{
  // modifying start and end points
  std::vector<double> start_pose, end_pose;
  if(input_traj.front()->getClosestJointPose(config_.seed_pose,*robot_model_ptr_,start_pose) &&
      input_traj.back()->getClosestJointPose(config_.seed_pose,*robot_model_ptr_,end_pose))
  {
    ROS_INFO_STREAM("Setting trajectory start and end to JointTrajectoryPts");
    descartes_core::TrajectoryPtPtr start_joint_point = descartes_core::TrajectoryPtPtr(
        new descartes_trajectory::JointTrajectoryPt(start_pose));
    descartes_core::TrajectoryPtPtr end_joint_point = descartes_core::TrajectoryPtPtr(
        new descartes_trajectory::JointTrajectoryPt(end_pose));

    input_traj[0] = start_joint_point;
    input_traj[input_traj.size() - 1 ] = end_joint_point;
  }
  else
  {
    ROS_ERROR_STREAM("Failed to find closest joint pose to seed pose at the start or end of trajectory");
    exit(-1);
  }

  // planning robot path
  if (planner_.planPath(input_traj))
  {
    ROS_INFO_STREAM("Valid path was found");
  }
  else
  {
    ROS_ERROR_STREAM("Could not solve for a valid path");
    exit(-1);
  }

  // retrieving robot path
  if(!planner_.getPath(output_path) || output_path.empty())
  {
    ROS_ERROR_STREAM("Failed to retrieve robot path");
  }
}

}


