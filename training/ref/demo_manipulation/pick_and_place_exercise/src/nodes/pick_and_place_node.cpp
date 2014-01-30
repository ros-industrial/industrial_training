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

  // marker publisher
  application.marker_publisher = nh.advertise<visualization_msgs::Marker>(
		  application.cfg.MARKER_TOPIC,1);

  // planning scene publisher
  application.planning_scene_publisher = nh.advertise<moveit_msgs::PlanningScene>(
  		application.cfg.PLANNING_SCENE_TOPIC,1);

  // moveit interface
  application.move_group_ptr = MoveGroupPtr(
		  new move_group_interface::MoveGroup(application.cfg.ARM_GROUP_NAME));

  // transform listener
  application.transform_listener_ptr = TransformListenerPtr(new tf::TransformListener());

  // marker publisher
  application.marker_publisher = nh.advertise<visualization_msgs::Marker>(
		  application.cfg.MARKER_TOPIC,1);

  // target recognition client
  application.target_recognition_client = nh.serviceClient<pick_and_place_exercise::GetTargetPose>(
		  application.cfg.TARGET_RECOGNITION_SERVICE);

  // grasp action client 
  application.grasp_action_client_ptr = GraspActionClientPtr(
		  new GraspActionClient(application.cfg.GRASP_ACTION_NAME,true));


  // waiting to establish connections
  while(ros::ok() &&
      !application.grasp_action_client_ptr->waitForServer(ros::Duration(2.0f)))
  {
    ROS_INFO_STREAM("Waiting for grasp action servers");
  }

  if(ros::ok() && !application.target_recognition_client.waitForExistence(ros::Duration(2.0f)))
  {
	  ROS_INFO_STREAM("Waiting for service'"<<application.cfg.TARGET_RECOGNITION_SERVICE<<"'");
  }


  /* ========================================*/
  /* Pick & Place Tasks                      */
  /* ========================================*/

  // clears the scene and gets a new obstacle map
  application.reset_world();

  // open the gripper (suction off)
  application.set_gripper(false);

  // move to a "clear" position
  application.move_to_wait_position();

  // get the box position and gets new octomap
  box_pose = application.detect_box_pick();

  // build a sequence of poses to "Pick" the box
  pick_poses = application.create_pick_moves(box_pose);

  // plan/execute the sequence of "pick" moves
  application.pickup_box(pick_poses,box_pose);

  // build a sequence of poses to "Place" the box
  place_poses = application.create_place_moves();

  // plan/execute the "place" moves
  application.place_box(place_poses);

  // move back to the "clear" position
  application.move_to_wait_position();

  return 0;
}
