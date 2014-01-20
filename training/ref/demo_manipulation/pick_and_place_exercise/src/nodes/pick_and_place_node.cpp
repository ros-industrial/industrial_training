/*
 * Manipulation Lab
 * pick_and_place_node.cpp
 *
 *  Created on: May 21, 2013
 *      Author: ros developer 
 */

#include <pick_and_place_exercise/pick_and_place.h>

pick_and_place_config cfg;  // global var
ros::Publisher marker_publisher; // publishes scene objects
ros::Publisher collision_object_publisher;
ros::Publisher attach_object_publisher; // publishes objects for use in path planning
ros::Publisher planning_scene_publisher; // scene publisher;

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

  // reading parameters
  if(cfg.init())
  {
    ROS_INFO_STREAM("Parameters successfully read");
  }
  else
  {
    ROS_ERROR_STREAM("Parameters not found");
    return 0;
  }

  // initializing marker publisher
  marker_publisher = nh.advertise<visualization_msgs::Marker>(cfg.MARKER_TOPIC,1);

  // moveit interface initialization
  move_group_interface::MoveGroup move_group(cfg.ARM_GROUP_NAME);
  move_group.setPlanningTime(10.0f);

  // planning scene publisher
  planning_scene_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene",1);

  // grasp action client initialization
  GraspActionClient grasp_action_client(cfg.GRASP_ACTION_NAME,true);

  // attached object publisher
  attach_object_publisher =
		  nh.advertise<moveit_msgs::AttachedCollisionObject>(cfg.ATTACHED_OBJECT_TOPIC,1);

  // collision object
  collision_object_publisher = nh.advertise<moveit_msgs::CollisionObject>(cfg.COLLISION_OBJECT_TOPIC,1);

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
  reset_world();

  // open the gripper (suction off)
  set_gripper(grasp_action_client, false);

  // move to a "clear" position
  move_to_wait_position(move_group);

  // get the box position from perception node
  box_pose = detect_box_pick();

  // build a sequence of poses to "Pick" the box
  pick_poses = create_pick_moves(tf_listener, box_pose);

  // plan/execute the sequence of "pick" moves
  pickup_box(move_group,grasp_action_client,pick_poses,box_pose);

  // build a sequence of poses to "Place" the box
  place_poses = create_place_moves(tf_listener);

  // plan/execute the "place" moves
  place_box(move_group,grasp_action_client,place_poses);

  // move back to the "clear" position
  move_to_wait_position(move_group);

  return 0;
}
