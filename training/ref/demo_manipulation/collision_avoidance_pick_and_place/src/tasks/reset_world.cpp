/*
 * open_gripper.cpp
 *
 *  Created on: Jun 3, 2013
 *      Author: ros-industrial
 */

#include <collision_avoidance_pick_and_place/pick_and_place.h>

/*    RESET WORLD
  Goal:
  Hints:
*/
void collision_avoidance_pick_and_place::PickAndPlace::reset_world(bool refresh_octomap)
{

	// clear entire scene
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	planning_scene::PlanningScene planning_scene(kinematic_model);
	planning_scene.setCurrentState(*move_group_ptr->getCurrentState());
	collision_detection::AllowedCollisionMatrix &acm = planning_scene.getAllowedCollisionMatrixNonConst();

	// modifying allowed collision matrix
	acm.setEntry(cfg.ATTACHED_OBJECT_LINK_NAME,"<octomap>",true);
	acm.setDefaultEntry(cfg.ATTACHED_OBJECT_LINK_NAME,true);

	// create planning scene message
	moveit_msgs::PlanningScene planning_scene_msg;
	planning_scene.getPlanningSceneMsg(planning_scene_msg);
	planning_scene_msg.is_diff = false;
	planning_scene_msg.world = moveit_msgs::PlanningSceneWorld();

	// publishing planning scene
	planning_scene_publisher.publish(planning_scene_msg);
	ros::Duration(1.0f).sleep();

	// get new sensor snapshot
	if(refresh_octomap)
	{
		detect_box_pick();
	}

}


