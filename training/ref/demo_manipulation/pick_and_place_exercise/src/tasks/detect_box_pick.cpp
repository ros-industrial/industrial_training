/*
 * detect_box_pick.cpp
 *
 *  Created on: Jun 3, 2013
 *      Author: ros-industrial
 */

#include <pick_and_place_exercise/pick_and_place.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/centroid.h>
#include <pcl_ros/transforms.h>

/* DETECTING BOX PICK POSE
  Goal:
    - Find the box's position in the world frame using the transform listener.
        * this transform is published by the kinect AR-tag perception node
    - Save the pose into 'box_pose'.

  Hints:
    - lookupTransform can also look "in the past".  Use Time=0 to get the most-recent transform.
    - tf::poseTFToMsg allows converting transforms into Pose messages
*/
geometry_msgs::Pose detect_box_pick()
{
  //ROS_ERROR_STREAM("detect_box_pick is not implemented yet.  Aborting."); exit(1);

  // creating request object
  pick_and_place_exercise::GetTargetPose srv;
  srv.request.shape = cfg.ATTACHED_COLLISION_OBJECT.object.primitives[0];
  srv.request.world_frame_id = cfg.WORLD_FRAME_ID;
  srv.request.ar_tag_frame_id = cfg.AR_TAG_FRAME_ID;

  if(!ros::service::waitForService(cfg.TARGET_RECOGNITION_SERVICE,10000))
  {
	  ROS_ERROR_STREAM("Service wait timeout on service '"<<cfg.TARGET_RECOGNITION_SERVICE<<"'");
  }

  // calling service
  geometry_msgs::Pose box_pose;
  if(ros::service::call(cfg.TARGET_RECOGNITION_SERVICE,srv))
  {
	  if(srv.response.succeeded)
	  {
		  box_pose = srv.response.target_pose;
	  }
	  else
	  {
		  ROS_ERROR_STREAM("target recognition failed, exiting");
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

  ros::Duration(1).sleep();

  return box_pose;
}

