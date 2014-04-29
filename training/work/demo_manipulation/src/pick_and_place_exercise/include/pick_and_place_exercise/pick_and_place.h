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
#include <object_manipulation_msgs/GraspHandPostureExecutionAction.h>
#include <tf/transform_listener.h>

#include <pick_and_place_exercise/pick_and_place_utilities.h>

// =============================== aliases ===============================
typedef actionlib::SimpleActionClient<object_manipulation_msgs::GraspHandPostureExecutionAction> GraspActionClient;

// =============================== global variables =====================================
extern pick_and_place_config cfg;

// =============================== Task Functions ===============================
void move_to_wait_position(move_group_interface::MoveGroup& move_group);
void set_gripper(GraspActionClient& grasp_action_client, bool do_grasp);
geometry_msgs::Pose detect_box_pick(tf::TransformListener &tf_listener);
std::vector<geometry_msgs::Pose> create_pick_moves(tf::TransformListener &tf_listener, geometry_msgs::Pose &box_pose);
void move_through_pick_poses(move_group_interface::MoveGroup& move_group, GraspActionClient& grasp_action_client,std::vector<geometry_msgs::Pose>& pick_poses);
std::vector<geometry_msgs::Pose> create_place_moves(tf::TransformListener& tf_listener);
void move_through_place_poses(move_group_interface::MoveGroup& move_group, GraspActionClient& grasp_action_client,std::vector<geometry_msgs::Pose>& place_poses);

#endif /* PICK_AND_PLACE_H_ */
