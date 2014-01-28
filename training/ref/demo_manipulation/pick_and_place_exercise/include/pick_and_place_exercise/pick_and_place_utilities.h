/*
 * pick_and_place_utilities.h
 *
 *  Created on: Jun 3, 2013
 *      Author: ros-industrial
 */

#ifndef PICK_AND_PLACE_UTILITIES_H_
#define PICK_AND_PLACE_UTILITIES_H_

#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/Constraints.h>
#include <visualization_msgs/Marker.h>
#include <pick_and_place_exercise/GetTargetPose.h>

std::vector<geometry_msgs::Pose> create_manipulation_poses(double retreat_dis,
		double approach_dis,const tf::Transform &target_tf);

std::vector<geometry_msgs::Pose> transform_from_tcp_to_wrist(tf::Transform tcp_to_wrist_tf,
		const std::vector<geometry_msgs::Pose> tcp_poses);

std::ostream& operator<<(std::ostream& os, const tf::Vector3 vec);
std::ostream& operator<<(std::ostream& os, const geometry_msgs::Point pt);

moveit_msgs::Constraints create_path_orientation_constraints(const geometry_msgs::Pose &goal_pose,
		float x_tolerance,float y_tolerance,float z_tolerance,std::string link_name);


class pick_and_place_config
{
public:

	// =============================== Parameters ===============================
  std::string ARM_GROUP_NAME;  // MoveIt Planning Group associated with the robot arm
  std::string TCP_LINK_NAME;   // Link / frame name for the suction gripper tool-tip
  std::string WRIST_LINK_NAME; // Link / frame name for the robot wrist tool-flange
  std::string ATTACHED_OBJECT_LINK_NAME; // attached object link in robot
  std::string WORLD_FRAME_ID;  // Frame name for the fixed world reference frame
  std::string AR_TAG_FRAME_ID;    // Frame name for the "AR Tag" mounted to the target box
  std::string HOME_POSE_NAME;  // Named pose for robot Home position (set in SRDF)
  std::string WAIT_POSE_NAME;  // Named pose for robot WAIT position (set in SRDF)
  tf::Vector3 BOX_SIZE;        // Size of the target box
  tf::Transform BOX_PLACE_TF;  // Transform from the WORLD frame to the PLACE location
  double RETREAT_DISTANCE;     // Distance to back away from pick/place pose after grasp/release
  double APPROACH_DISTANCE;    // Distance to stand off from pick/place pose before grasp/release

  // =============================== Variables ===============================
  std::string GRASP_ACTION_NAME;  // Action name used to control suction gripper
  std::string MARKER_TOPIC; // Topic for publishing visualization of attached object.
  std::string TARGET_RECOGNITION_SERVICE; // service for requesting box pick pose

  // =============================== Messages ===============================
  visualization_msgs::Marker MARKER_MESSAGE; // visual representation of target object

  pick_and_place_config()
  {
    ARM_GROUP_NAME  = "manipulator";
    TCP_LINK_NAME   = "tcp_frame";
    MARKER_TOPIC = "pick_and_place_marker";
    TARGET_RECOGNITION_SERVICE = "target_recognition";
    WRIST_LINK_NAME = "ee_link";
    ATTACHED_OBJECT_LINK_NAME = "attached_object_link";
    WORLD_FRAME_ID  = "world_frame";
    HOME_POSE_NAME  = "home";
    WAIT_POSE_NAME  = "wait";
    AR_TAG_FRAME_ID    = "ar_frame";
    GRASP_ACTION_NAME = "grasp_execution_action";
    BOX_SIZE        = tf::Vector3(0.1f, 0.1f, 0.1f);
    BOX_PLACE_TF    = tf::Transform(tf::Quaternion::getIdentity(), tf::Vector3(-0.8f,-0.2f,BOX_SIZE.getZ()));
    RETREAT_DISTANCE  = 0.05f;
    APPROACH_DISTANCE = 0.05f;
  }

  bool init();
};

#endif /* PICK_AND_PLACE_UTILITIES_H_ */
