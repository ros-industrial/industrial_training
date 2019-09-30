/*
 * move_robot.cpp
 *
 *  Created on: Sep 24, 2019
 *      Author: Jorge Nicho
 */

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include "myworkcell_core/MoveToPose.h"
#include "myworkcell_core/ExecuteTrajectory.h"

static const std::string JOINT_TRAJECTORY_ACTION = "joint_trajectory_action";
static const std::string MOVE_TO_POSE_SERVICE = "move_to_pose";
static const std::string EXECUTE_TRAJECTORY_SERVICE = "execute_trajectory";

class MoveRobot
{
public:
  MoveRobot():
    ac_(JOINT_TRAJECTORY_ACTION, true)
  {

  }

  ~MoveRobot(){

  }

  bool run()
  {
    // loading parameter
    ros::NodeHandle ph("~");
    if(!ph.getParam("group",group_))
    {
      ROS_ERROR("Failed to get parameter");
      return false;
    }

    // advertise service(s)
    ros::NodeHandle nh;
    move_to_pose_server_ = nh.advertiseService(MOVE_TO_POSE_SERVICE, &MoveRobot::moveToPoseCallback, this);
    execute_traj_server_ = nh.advertiseService(EXECUTE_TRAJECTORY_SERVICE, &MoveRobot::executeTrajCallback, this);

    // wait for action
    if(!ac_.waitForServer(ros::Duration(1.0)))
    {
      ROS_ERROR("Action %s is not available",JOINT_TRAJECTORY_ACTION.c_str());
      return false;
    }

    return true;
  }

  bool executeTrajCallback(myworkcell_core::ExecuteTrajectoryRequest& req, myworkcell_core::ExecuteTrajectoryResponse& res)
  {
    ROS_INFO("Got cart path, executing");
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = req.trajectory;
    ac_.sendGoal(goal);
    res.succeeded = ac_.waitForResult();
    ROS_INFO_COND(res.succeeded,"Done");
    ROS_ERROR_COND(!res.succeeded,"Execution failed");
    return true;
  }

  bool moveToPoseCallback(myworkcell_core::MoveToPoseRequest& req, myworkcell_core::MoveToPoseResponse& res)
  {
    moveit::planning_interface::MoveGroupInterface move_group(group_);

    // move robot
    move_group.setPoseReferenceFrame(req.pose.header.frame_id);
    move_group.setPoseTarget(req.pose.pose);
    moveit_msgs::MoveItErrorCodes error = move_group.move();

    res.succeeded = error.val == error.SUCCESS;
    return true;
  }

protected:

  std::string group_;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_;
  ros::ServiceServer move_to_pose_server_;
  ros::ServiceServer execute_traj_server_;
};

int main(int argc, char** argv)
{
  ros::init(argc,argv,"move_robot");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();
  MoveRobot app;
  if(!app.run())
  {
    ros::shutdown();
    return -1;
  }
  ros::waitForShutdown();
  return 0;
}


