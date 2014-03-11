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

namespace collision_avoidance_pick_and_place
{
void PickAndPlace::set_attached_object(bool attach, const geometry_msgs::Pose &pose,moveit_msgs::RobotState &robot_state)
{
  //ROS_ERROR_STREAM("set_attached_object is not implemented yet.  Aborting."); exit(1);

	// get robot state
	robot_state::RobotStatePtr current_state= move_group_ptr->getCurrentState();

	if(attach)
	{

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
		current_state->attachBody(cfg.ATTACHED_OBJECT_LINK_NAME,shapes_array,pose_array,cfg.TOUCH_LINKS,cfg.TCP_LINK_NAME);

		// update box marker
		cfg.MARKER_MESSAGE.header.frame_id = cfg.TCP_LINK_NAME;
		cfg.MARKER_MESSAGE.pose = cfg.TCP_TO_BOX_POSE;
	}
	else
	{

		// detaching
		if(current_state->hasAttachedBody(cfg.ATTACHED_OBJECT_LINK_NAME))
				current_state->clearAttachedBody(cfg.ATTACHED_OBJECT_LINK_NAME);
	}

	// save robot state data
	robot_state::robotStateToRobotStateMsg(*current_state,robot_state);
}

}


