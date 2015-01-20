#include <collision_avoidance_pick_and_place/pick_and_place.h>

using namespace collision_avoidance_pick_and_place;

// =============================== Main Thread ===============================
int main(int argc,char** argv)
{
  geometry_msgs::Pose box_pose;
  std::vector<geometry_msgs::Pose> pick_poses, place_poses;

  /* =========================================================================================*/
  /*	INITIALIZING ROS NODE
      Goal:
      - Observe all steps needed to properly initialize a ros node.
      - Look into the 'cfg' member of PickAndPlace to take notice of the parameters that
        are available for the rest of the program. */
  /* =========================================================================================*/

  // ros initialization
  ros::init(argc,argv,"pick_and_place_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // creating pick and place application instance
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

  // motion plan client
  application.motion_plan_client = nh.serviceClient<moveit_msgs::GetMotionPlan>(application.cfg.MOTION_PLAN_SERVICE);

  // transform listener
  application.transform_listener_ptr = TransformListenerPtr(new tf::TransformListener());

  // marker publisher (rviz visualization)
  application.marker_publisher = nh.advertise<visualization_msgs::Marker>(
		  application.cfg.MARKER_TOPIC,1);

  // target recognition client (perception)
  application.target_recognition_client = nh.serviceClient<collision_avoidance_pick_and_place::GetTargetPose>(
		  application.cfg.TARGET_RECOGNITION_SERVICE);

  // grasp action client (vacuum gripper)
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

  // move to a "clear" position
  application.move_to_wait_position();

  // turn off vacuum gripper
  application.set_gripper(false);

  // get the box position and orientation
  box_pose = application.detect_box_pick();

  // build a sequence of poses to "pick" the box
  pick_poses = application.create_pick_moves(box_pose);

  // plan/execute the sequence of "pick" moves
  application.pickup_box(pick_poses,box_pose);

  // build a sequence of poses to "place" the box
  place_poses = application.create_place_moves();

  // plan/execute the "place" moves
  application.place_box(place_poses,box_pose);

  // move back to the "clear" position
  application.move_to_wait_position();

  return 0;
}
