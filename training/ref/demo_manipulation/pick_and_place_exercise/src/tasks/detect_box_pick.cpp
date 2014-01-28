/*
 * detect_box_pick.cpp
 *
 *  Created on: Jun 3, 2013
 *      Author: ros-industrial
 */

#include <pick_and_place_exercise/pick_and_place.h>

/* DETECTING BOX PICK POSE
  Goal:
    - Find the box's position in the world frame using the transform listener.
        * this transform is published by the kinect AR-tag perception node
    - Save the pose into 'box_pose'.

  Hints:
    - lookupTransform can also look "in the past".  Use Time=0 to get the most-recent transform.
    - tf::poseTFToMsg allows converting transforms into Pose messages
*/
geometry_msgs::Pose PickAndPlace::detect_box_pick()
{
  //ROS_ERROR_STREAM("detect_box_pick is not implemented yet.  Aborting."); exit(1);

  // creating request object
  pick_and_place_exercise::GetTargetPose srv;
  srv.request.shape = cfg.ATTACHED_COLLISION_OBJECT.object.primitives[0];
  srv.request.world_frame_id = cfg.WORLD_FRAME_ID;
  srv.request.ar_tag_frame_id = cfg.AR_TAG_FRAME_ID;
  geometry_msgs::Pose place_pose;
  tf::poseTFToMsg(cfg.BOX_PLACE_TF,place_pose);
  srv.request.remove_at_poses.push_back(place_pose);

  if(!ros::service::waitForService(cfg.TARGET_RECOGNITION_SERVICE,10000))
  {
	  ROS_ERROR_STREAM("Service wait timeout on service '"<<cfg.TARGET_RECOGNITION_SERVICE<<"'");
  }

  // calling service
  geometry_msgs::Pose box_pose;
  if(target_recognition_client.call(srv))
  {
	  if(srv.response.succeeded)
	  {
		  box_pose = srv.response.target_pose;
		  ROS_INFO_STREAM("target recognition succeeded");
	  }
	  else
	  {
		  ROS_ERROR_STREAM("target recognition failed");
		  return box_pose;

	  }
  }
  else
  {
	  ROS_ERROR_STREAM("Service call for target recognition failed with response '"<<
			  (srv.response.succeeded ?"SUCCESS":"FAILURE")
					  <<"', exiting");
	  exit(0);
  }

	// creating collision and visualization messages
    moveit_msgs::PlanningScene planning_scene;
	moveit_msgs::CollisionObject  col_obj;// = cfg.ATTACHED_COLLISION_OBJECT.object;
	visualization_msgs::Marker marker = cfg.MARKER_MESSAGE;

	// updating pose of collision object
	col_obj.id = cfg.ATTACHED_COLLISION_OBJECT.object.id;
	col_obj.header.frame_id = cfg.WORLD_FRAME_ID;
	col_obj.primitives = cfg.ATTACHED_COLLISION_OBJECT.object.primitives;
	col_obj.primitive_poses.push_back(box_pose);
	col_obj.primitive_poses[0].position.z = 0.5f *col_obj.primitive_poses[0].position.z;
	marker.header.frame_id = cfg.WORLD_FRAME_ID;
	marker.pose = col_obj.primitive_poses[0];

	// set object operation
	col_obj.operation = moveit_msgs::CollisionObject::ADD;
	marker.action = visualization_msgs::Marker::ADD;

	// modifying collision matrix
	planning_scene.allowed_collision_matrix.default_entry_values.push_back(false);
	planning_scene.is_diff = true;
	// filling planning scene

	// publishing messages
	marker_publisher.publish(marker);
	//planning_scene_publisher.publish(planning_scene);
	//collision_object_publisher.publish(col_obj);

	ros::Duration(2.0f).sleep();

	return box_pose;
}

