/*
 * open_gripper.cpp
 *
 *  Created on: Jun 3, 2013
 *      Author: ros-industrial
 */

#include <pick_and_place_exercise/pick_and_place.h>

/*    SET ATTACHED OBJECT
  Goal:
    - Attaches or detaches target to arms end-effector link.
    - Publishes object marker for visualization.
  Hints:
*/
void PickAndPlace::set_attached_object(bool attach, const geometry_msgs::Pose &pose)
{
  //ROS_ERROR_STREAM("set_attached_object is not implemented yet.  Aborting."); exit(1);

	moveit_msgs::PlanningScene planning_scene;
	moveit_msgs::CollisionObject remove_obj;
	if(attach)
	{
		cfg.ATTACHED_COLLISION_OBJECT.object.operation = moveit_msgs::CollisionObject::ADD;
		cfg.ATTACHED_COLLISION_OBJECT.object.header.frame_id = cfg.TCP_LINK_NAME;

		remove_obj.id= cfg.ATTACHED_COLLISION_OBJECT.object.id;
		remove_obj.header.frame_id = cfg.WORLD_FRAME_ID;
		remove_obj.operation = moveit_msgs::CollisionObject::REMOVE;
		cfg.MARKER_MESSAGE.action = visualization_msgs::Marker::ADD;

		// updating orientation
		geometry_msgs::Quaternion q = pose.orientation;
		q.x = -q.x;
		q.y = -q.y;
		q.z = -q.z;

		cfg.ATTACHED_COLLISION_OBJECT.object.primitive_poses[0].orientation = q;
		cfg.MARKER_MESSAGE.pose.orientation = q;

		// modifying collision matrix
		planning_scene.allowed_collision_matrix.default_entry_names.push_back(cfg.ATTACHED_LINK_NAME);
		planning_scene.allowed_collision_matrix.default_entry_values.push_back(true);

		// updating planning scene message
		planning_scene.world.collision_objects.push_back(remove_obj);
		planning_scene.robot_state.attached_collision_objects.push_back(cfg.ATTACHED_COLLISION_OBJECT);
		planning_scene.robot_state.is_diff = true;
		planning_scene.is_diff = true;


	}
	else
	{
		cfg.ATTACHED_COLLISION_OBJECT.object.operation = moveit_msgs::CollisionObject::REMOVE;
		cfg.MARKER_MESSAGE.action = visualization_msgs::Marker::DELETE;

		// updating planning scene message
		planning_scene.world.collision_objects.push_back(cfg.ATTACHED_COLLISION_OBJECT.object);
		planning_scene.robot_state.attached_collision_objects.push_back(cfg.ATTACHED_COLLISION_OBJECT);
		planning_scene.robot_state.is_diff = true;
		planning_scene.allowed_collision_matrix.default_entry_names.push_back(cfg.ATTACHED_LINK_NAME);
		planning_scene.allowed_collision_matrix.default_entry_values.push_back(false);
		planning_scene.is_diff = true;

	}

	marker_publisher.publish(cfg.MARKER_MESSAGE);

	//ros::Duration(2.0f).sleep();



}


