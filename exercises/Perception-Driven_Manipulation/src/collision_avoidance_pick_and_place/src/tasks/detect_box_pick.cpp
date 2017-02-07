#include <collision_avoidance_pick_and_place/pick_and_place.h>

/* DETECTING BOX PICK POSE
  Goal:
    - Find the box's position in the world frame using the transform listener.
        * this transform is published by the kinect AR-tag perception node
    - Save the pose into 'box_pose'.

  Hints:
    - lookupTransform can also look "in the past".  Use Time=0 to get the most-recent transform.
    - tf::poseTFToMsg allows converting transforms into Pose messages
*/
geometry_msgs::Pose collision_avoidance_pick_and_place::PickAndPlace::detect_box_pick()
{
  //ROS_ERROR_STREAM("detect_box_pick is not implemented yet.  Aborting."); exit(1);

  // creating shape for recognition
  shape_msgs::SolidPrimitive shape;
  shape.type = shape_msgs::SolidPrimitive::BOX;
  shape.dimensions.resize(3);
  shape.dimensions[0] = cfg.BOX_SIZE.getX();
  shape.dimensions[1] = cfg.BOX_SIZE.getY();
  shape.dimensions[2] = cfg.BOX_SIZE.getZ();

  // creating request object
  collision_avoidance_pick_and_place::GetTargetPose srv;
  srv.request.shape = shape;
  srv.request.world_frame_id = cfg.WORLD_FRAME_ID;
  srv.request.ar_tag_frame_id = cfg.AR_TAG_FRAME_ID;
  geometry_msgs::Pose place_pose;
  tf::poseTFToMsg(cfg.BOX_PLACE_TF,place_pose);
  srv.request.remove_at_poses.push_back(place_pose);

  //
  /* Fill Code:
   * Goal:
   * - Call target recognition service and save results.
   * Hint:
   * - Use the service response member to access the
   * 	detected pose "srv.response.target_pose".
   * - Assign the target_pose in the response to the box_pose variable in
   * 	order to save the results.
   * 	*/
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
		  exit(0);

	  }
  }
  else
  {
	  ROS_ERROR_STREAM("Service call for target recognition failed with response '"<<
			  (srv.response.succeeded ?"SUCCESS":"FAILURE")
					  <<"', exiting");
	  exit(0);
  }

  // updating box marker for visualization in rviz
	visualization_msgs::Marker marker = cfg.MARKER_MESSAGE;
	cfg.MARKER_MESSAGE.header.frame_id = cfg.WORLD_FRAME_ID;
	cfg.MARKER_MESSAGE.pose = box_pose;
	cfg.MARKER_MESSAGE.pose.position.z = box_pose.position.z - 0.5f*cfg.BOX_SIZE.z();

	show_box(true);

	return box_pose;
}

