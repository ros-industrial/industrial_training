/*
 * open_gripper.cpp
 *
 *  Created on: Jun 3, 2013
 *      Author: ros-industrial
 */

#include <collision_avoidance_pick_and_place/pick_and_place.h>

/*    SET ATTACHED OBJECT
  Goal:
    - Attaches or detaches target to arms end-effector link.
    - Publishes object marker for visualization.
  Hints:
*/
void collision_avoidance_pick_and_place::PickAndPlace::set_attached_object(bool attach, const geometry_msgs::Pose &pose)
{
  //ROS_ERROR_STREAM("set_attached_object is not implemented yet.  Aborting."); exit(1);

	// get robot state
	robot_state::RobotStatePtr current_state= move_group_ptr->getCurrentState();

	if(attach)
	{
		// updating orientation
		geometry_msgs::Quaternion q = pose.orientation;
		q.x = -q.x;
		q.y = -q.y;
		q.z = -q.z;
		cfg.MARKER_MESSAGE.pose.orientation = q;
		cfg.ATTACHED_OBJECT.primitive_poses[0].orientation = q;

		// constructing shape
		std::vector<shapes::ShapeConstPtr> shapes_array;
		shapes::ShapeConstPtr shape( shapes::constructShapeFromMsg( cfg.ATTACHED_OBJECT.primitives[0]));
		shapes_array.push_back(shape);

		// constructing pose
		tf::Transform attached_tf;
		tf::poseMsgToTF(cfg.ATTACHED_OBJECT.primitive_poses[0],attached_tf);
		EigenSTL::vector_Affine3d pose_array(1);
		tf::transformTFToEigen(attached_tf,pose_array[0]);

		// attaching
		current_state->attachBody(cfg.ATTACHED_OBJECT_LINK_NAME,shapes_array,pose_array,std::vector<std::string>(),cfg.TCP_LINK_NAME);
	}
	else
	{

		// detaching
		current_state->clearAttachedBodies(cfg.ATTACHED_OBJECT_LINK_NAME);
	}

	// setting moveit interface start state
	move_group_ptr->setStartState(*current_state);

	// updating marker action
	cfg.MARKER_MESSAGE.action =
			attach ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;

	// publish marker
	marker_publisher.publish(cfg.MARKER_MESSAGE);

	ros::Duration(1.0f).sleep();

}


