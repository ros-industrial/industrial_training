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
void set_attached_object(bool attach, const geometry_msgs::Pose &pose)
{
  //ROS_ERROR_STREAM("set_attached_object is not implemented yet.  Aborting."); exit(1);

	// update pose

	if(attach)
	{
		cfg.ATTACHED_COLLISION_OBJECT.object.operation = moveit_msgs::CollisionObject::ADD;
		cfg.MARKER_MESSAGE.action = visualization_msgs::Marker::ADD;

		// updating orientation
		geometry_msgs::Quaternion q = pose.orientation;
		q.x = -q.x;
		q.y = -q.y;
		q.z = -q.z;

		cfg.ATTACHED_COLLISION_OBJECT.object.primitive_poses[0].orientation = q;
		cfg.MARKER_MESSAGE.pose.orientation = q;
	}
	else
	{
		cfg.ATTACHED_COLLISION_OBJECT.object.operation = moveit_msgs::CollisionObject::REMOVE;
		cfg.MARKER_MESSAGE.action = visualization_msgs::Marker::DELETE;
	}

	marker_publisher.publish(cfg.MARKER_MESSAGE);
	attach_object_publisher.publish(cfg.ATTACHED_COLLISION_OBJECT);

}


