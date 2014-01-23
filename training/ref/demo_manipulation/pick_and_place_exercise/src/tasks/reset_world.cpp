/*
 * open_gripper.cpp
 *
 *  Created on: Jun 3, 2013
 *      Author: ros-industrial
 */

#include <pick_and_place_exercise/pick_and_place.h>

/*    SET OBJECT IN WORLD
  Goal:
    - Attaches or detaches target to arms end-effector link.
    - Publishes object marker for visualization.
  Hints:
*/
void PickAndPlace::reset_world()
{
  //ROS_ERROR_STREAM("set_attached_object is not implemented yet.  Aborting."); exit(1);

	// planning scene message
	moveit_msgs::PlanningScene planning_scene;

	// removing attached objects from robot
	set_attached_object(false);

	// detecting object pose
	geometry_msgs::Pose pose = detect_box_pick();

	// creating collision and visualization messages
	moveit_msgs::CollisionObject  col_obj;// = cfg.ATTACHED_COLLISION_OBJECT.object;
	visualization_msgs::Marker marker = cfg.MARKER_MESSAGE;

	// updating pose of collision object
	col_obj.id = cfg.ATTACHED_COLLISION_OBJECT.object.id;
	col_obj.operation = col_obj.ADD;
	col_obj.header.frame_id = cfg.WORLD_FRAME_ID;
	col_obj.primitives = cfg.ATTACHED_COLLISION_OBJECT.object.primitives;
	col_obj.primitive_poses.push_back(pose);
	col_obj.primitive_poses[0].position.z = 0.5f *col_obj.primitive_poses[0].position.z;
	marker.header.frame_id = cfg.WORLD_FRAME_ID;
	marker.pose = col_obj.primitive_poses[0];

	// set object operation
	col_obj.operation = moveit_msgs::CollisionObject::ADD;
	marker.action = visualization_msgs::Marker::ADD;

	// filling planning scene
	planning_scene.world.collision_objects.push_back(col_obj);
	planning_scene.is_diff = true;

	// publishing messages
	marker_publisher.publish(marker);
	//planning_scene_publisher.publish(planning_scene);

	ros::Duration(2.0f).sleep();

}


