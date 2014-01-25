	/*
 * Manipulation Lab
 * pick_and_place_node.cpp
 *
 *  Created on: May 21, 2013
 *      Author: ros developer 
 */

#include <pick_and_place_exercise/pick_and_place.h>

// =============================== Main Thread ===============================
int main(int argc,char** argv)
{
  geometry_msgs::Pose box_pose;
  std::vector<geometry_msgs::Pose> pick_poses, place_poses;

  /* =========================================================================================*/
  /*	INITIALIZING ROS NODE
      Goal:
      - Observe all steps needed to properly initialize a ros node.
      - Look into the 'cfg' global var to take notice of the parameters that
        are available for the rest of the program. */
  /* =========================================================================================*/

  // ros initialization
  ros::init(argc,argv,"pick_and_place_node");
  ros::NodeHandle nh;
  tf::TransformListener tf_listener; // queries tf to find transforms
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // creating pick and place instance
  PickAndPlace application;

  // reading parameters
  if(application.cfg.init())
  {
    ROS_INFO_STREAM("Parameters successfully read");
  }
  else
  {
    ROS_ERROR_STREAM("Parameters not found");
    return 0;
  }

  // initializing marker publisher
  application.marker_publisher = nh.advertise<visualization_msgs::Marker>(
		  application.cfg.MARKER_TOPIC,1);

  // moveit interface initialization
  move_group_interface::MoveGroup move_group(application.cfg.ARM_GROUP_NAME);
  move_group.setStartStateToCurrentState();
  move_group.setPlanningTime(10.0f);


  // initializing marker publisher
  application.marker_publisher = nh.advertise<visualization_msgs::Marker>(
		  application.cfg.MARKER_TOPIC,1);

  // initializing target recognition client
  application.target_recognition_client = nh.serviceClient<pick_and_place_exercise::GetTargetPose>(
		  application.cfg.TARGET_RECOGNITION_SERVICE);

  // planning scene publisher
  application.planning_scene_publisher = nh.advertise<moveit_msgs::PlanningScene>(
		  application.cfg.PLANNING_SCENE_TOPIC,1);

  // grasp action client initialization
  GraspActionClient grasp_action_client(application.cfg.GRASP_ACTION_NAME,true);

  // attached object publisher
  application.attach_object_publisher =
		  nh.advertise<moveit_msgs::AttachedCollisionObject>(
				  application.cfg.ATTACHED_OBJECT_TOPIC,1);

  // collision object
  application.collision_object_publisher =
		  nh.advertise<moveit_msgs::CollisionObject>(application.cfg.COLLISION_OBJECT_TOPIC,1);

  // waiting to establish connections
  while(ros::ok() &&
      !grasp_action_client.waitForServer(ros::Duration(2.0f)))
  {
    ROS_INFO_STREAM("Waiting for servers");
  }


  /* ========================================*/
  /* Pick & Place Tasks                      */
  /* ========================================*/

  // updates the obstacle map
  application.reset_world();

  // open the gripper (suction off)
  application.set_gripper(grasp_action_client, false);

  // move to a "clear" position
  application.move_to_wait_position(move_group);

  // get the box position from perception node
  box_pose = application.detect_box_pick();

  // build a sequence of poses to "Pick" the box
  pick_poses = application.create_pick_moves(tf_listener, box_pose);

  // plan/execute the sequence of "pick" moves
  application.pickup_box(move_group,grasp_action_client,pick_poses,box_pose);

  // build a sequence of poses to "Place" the box
  place_poses = application.create_place_moves(tf_listener);

  // plan/execute the "place" moves
  application.place_box(move_group,grasp_action_client,place_poses);

  // move back to the "clear" position
  application.move_to_wait_position(move_group);

  return 0;
}
