#ifndef PICK_AND_PLACE_H_
#define PICK_AND_PLACE_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/PlanningScene.h>
#include <object_manipulation_msgs/GraspHandPostureExecutionAction.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <collision_avoidance_pick_and_place/pick_and_place_utilities.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <geometric_shapes/shape_operations.h>

// =============================== aliases ===============================
typedef actionlib::SimpleActionClient<object_manipulation_msgs::GraspHandPostureExecutionAction> GraspActionClient;
typedef boost::shared_ptr<GraspActionClient> GraspActionClientPtr;
typedef boost::shared_ptr<move_group_interface::MoveGroup> MoveGroupPtr;
typedef boost::shared_ptr<tf::TransformListener> TransformListenerPtr;

namespace collision_avoidance_pick_and_place
{
	class PickAndPlace
	{
	public:
	// =============================== constructor =====================================
		PickAndPlace()
		{

		}

	// =============================== global members =====================================
		pick_and_place_config cfg;
		ros::Publisher marker_publisher;
		ros::Publisher planning_scene_publisher;
		ros::ServiceClient target_recognition_client;
		ros::ServiceClient motion_plan_client;
		GraspActionClientPtr grasp_action_client_ptr;
		MoveGroupPtr move_group_ptr;
		TransformListenerPtr transform_listener_ptr;

	// =============================== Task Functions ===============================
		void move_to_wait_position();

		void set_gripper(bool do_grasp);

		void set_attached_object(bool attach,
				const geometry_msgs::Pose &pose,moveit_msgs::RobotState &robot_state);

		void reset_world(bool refresh_octomap = true);

		geometry_msgs::Pose detect_box_pick();

		std::vector<geometry_msgs::Pose> create_pick_moves(geometry_msgs::Pose &box_pose);

		std::vector<geometry_msgs::Pose> create_place_moves();

		void pickup_box(std::vector<geometry_msgs::Pose>& pick_poses,const geometry_msgs::Pose& box_pose);

		void place_box(std::vector<geometry_msgs::Pose>& place_poses,const geometry_msgs::Pose& box_pose);


		bool create_motion_plan(const geometry_msgs::Pose &pose_target,
				const moveit_msgs::RobotState &start_robot_state,move_group_interface::MoveGroup::Plan &plan);

		void show_box(bool show=true)
		{
			// updating marker action
			cfg.MARKER_MESSAGE.action =
					show ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;

			// publish messages
			marker_publisher.publish(cfg.MARKER_MESSAGE);
		}

	};
}

#endif /* PICK_AND_PLACE_H_ */
