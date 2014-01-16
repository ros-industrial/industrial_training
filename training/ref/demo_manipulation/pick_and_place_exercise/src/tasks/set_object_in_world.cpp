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
void set_object_in_world(bool add, const geometry_msgs::Pose &pose)
{
  //ROS_ERROR_STREAM("set_attached_object is not implemented yet.  Aborting."); exit(1);

	// update pose
	moveit_msgs::AttachedCollisionObject  col_obj = cfg.ATTACHED_COLLISION_OBJECT;
	visualization_msgs::Marker marker = cfg.MARKER_MESSAGE;

	if(add)
	{
		col_obj.object.header.frame_id = cfg.WORLD_FRAME_ID;
		col_obj.object.primitive_poses[0] = pose;
		col_obj.object.primitive_poses[0].position.z = pose.position.z - 0.5f*cfg.BOX_SIZE.z();
		//col_obj.link_name = cfg.WORLD_FRAME_ID;
		marker.header.frame_id = cfg.WORLD_FRAME_ID;
		marker.pose = col_obj.object.primitive_poses[0];

		col_obj.object.operation = moveit_msgs::CollisionObject::ADD;
		//col_obj.touch_links.push_back("gripper_body");
		marker.action = visualization_msgs::Marker::ADD;
	}
	else
	{
		col_obj.object.operation = moveit_msgs::CollisionObject::REMOVE;
		marker.action = visualization_msgs::Marker::DELETE;
	}

	marker_publisher.publish(marker);
	attach_object_publisher.publish(col_obj);

}


