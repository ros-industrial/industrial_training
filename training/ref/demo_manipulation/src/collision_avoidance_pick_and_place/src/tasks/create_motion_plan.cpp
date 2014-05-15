/*
 * create_motion_plan.cpp
 *
 *  Created on: Feb 20, 2014
 *      Author: ros-industrial
 */

#include <collision_avoidance_pick_and_place/pick_and_place.h>

/* CREATE MOTION PLAN
  Goal:
    - Find the box's position in the world frame using the transform listener.
        * this transform is published by the kinect AR-tag perception node
    - Save the pose into 'box_pose'.

  Hints:
    - lookupTransform can also look "in the past".  Use Time=0 to get the most-recent transform.
    - tf::poseTFToMsg allows converting transforms into Pose messages
*/

namespace collision_avoidance_pick_and_place
{

bool PickAndPlace::create_motion_plan(const geometry_msgs::Pose &pose_target,
		const moveit_msgs::RobotState &start_robot_state,move_group_interface::MoveGroup::Plan &plan)
{
	// creating motion plan
	moveit_msgs::GetMotionPlan motion_plan;
	moveit_msgs::MotionPlanRequest &req = motion_plan.request.motion_plan_request;
	moveit_msgs::MotionPlanResponse &res = motion_plan.response.motion_plan_response;

	// filling motion plan request
	std::vector<double> position_tolerances(3,0.01f);
	std::vector<double> orientation_tolerances(3,0.01f);
	geometry_msgs::PoseStamped p;
	p.header.frame_id = cfg.WORLD_FRAME_ID;
	p.pose = pose_target;
	moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(cfg.WRIST_LINK_NAME,p,position_tolerances,
			orientation_tolerances);

	req.start_state = start_robot_state;
	req.start_state.is_diff = true;
	req.group_name = cfg.ARM_GROUP_NAME;
	req.goal_constraints.push_back(pose_goal);
	req.allowed_planning_time = 60.0f;
	req.num_planning_attempts = 10;

	// request motion plan
	bool success = false;
	if(motion_plan_client.call(motion_plan) && res.error_code.val == res.error_code.SUCCESS)
	{
		plan.start_state_ = res.trajectory_start;
		plan.trajectory_ = res.trajectory;
		success = true;
	}

	return success;
}

}




