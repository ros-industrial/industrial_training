/*
 * pick_and_place_headers.h
 *
 *  Created on: Jun 3, 2013
 *      Author: ros-industrial
 */

#ifndef PICK_AND_PLACE_H_
#define PICK_AND_PLACE_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/PlanningScene.h>
#include <object_manipulation_msgs/GraspHandPostureExecutionAction.h>
#include <tf/transform_listener.h>
#include <pick_and_place_exercise/pick_and_place_utilities.h>

// =============================== aliases ===============================
typedef actionlib::SimpleActionClient<object_manipulation_msgs::GraspHandPostureExecutionAction> GraspActionClient;


class PickAndPlace
{
public:
// =============================== constructor =====================================
	PickAndPlace()
	{

	}

// =============================== global members =====================================
	pick_and_place_config cfg;
	ros::Publisher marker_publisher;
	ros::Publisher collision_object_publisher;
	ros::Publisher attach_object_publisher;
	ros::Publisher planning_scene_publisher;
	ros::ServiceClient target_recognition_client;

// =============================== Task Functions ===============================
	void move_to_wait_position(move_group_interface::MoveGroup& move_group);

	void set_gripper(GraspActionClient& grasp_action_client, bool do_grasp);

	void set_attached_object(bool attach,
			const geometry_msgs::Pose &pose = geometry_msgs::Pose());

	void reset_world();

	geometry_msgs::Pose detect_box_pick();

	std::vector<geometry_msgs::Pose> create_pick_moves(tf::TransformListener &tf_listener,
			geometry_msgs::Pose &box_pose);

	void move_through_pick_poses(move_group_interface::MoveGroup& move_group,
			GraspActionClient& grasp_action_client,
			std::vector<geometry_msgs::Pose>& pick_poses);

	std::vector<geometry_msgs::Pose> create_place_moves(tf::TransformListener& tf_listener);

	void move_through_place_poses(move_group_interface::MoveGroup& move_group,
			GraspActionClient& grasp_action_client,
			std::vector<geometry_msgs::Pose>& place_poses);

	void pickup_box(move_group_interface::MoveGroup& move_group,
			GraspActionClient& grasp_action_client,
			std::vector<geometry_msgs::Pose>& pick_poses,const geometry_msgs::Pose& box_pose);

	void place_box(move_group_interface::MoveGroup& move_group,
			GraspActionClient& grasp_action_client,
			std::vector<geometry_msgs::Pose>& place_poses);

};

#endif /* PICK_AND_PLACE_H_ */
