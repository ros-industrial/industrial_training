#include <ros/ros.h>
#include <myworkcell_core/LocalizePart.h>
#include <tf/tf.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <myworkcell_core/PlanCartesianPath.h>

class ScanNPlan
{
public:
  /**
  * @brief The ScanNPlan class is a service client of a node that provides part
  * localization and a node that provides path planning.  The ScanNPLan class
  * recieves these service response data computes transforms and publishes
  * commands to the robot via actionlib
  * @param nh is the NodeHandle the ScanNPlan class will use to subscribe to
  * the LocalizePart and PlanCartesianPath services.
  */
  ScanNPlan(ros::NodeHandle& nh) : ac_("joint_trajectory_action", true)
  {
    vision_client_ = nh.serviceClient<myworkcell_core::LocalizePart>("localize_part");
    cartesian_client_ = nh.serviceClient<myworkcell_core::PlanCartesianPath>("plan_path");
  }

  /**
   * @brief start performs the robot alorithms functions of the ScanNPlan of
   * the node. The start method makes a service request for a transform that
   * localizes the part.  The start method moves the "manipulator"
   * move group to the localization target.  The start method requests
   * a cartesian path based on the localization target.  The start method
   * sends the cartesian path to the actionlib client for execution, bypassig
   * MoveIt!
   * @param base_frame is a string that specifies the reference frame
   * coordinate system.
   */
  void start(const std::string& base_frame)
  {
    ROS_INFO("Attempting to localize part");

    // Localize the part
    myworkcell_core::LocalizePart srv;
    srv.request.base_frame = base_frame;
    ROS_INFO_STREAM("Requesting pose in base frame: " << base_frame);

    if (!vision_client_.call(srv))
    {
      ROS_ERROR("Could not localize part");
      return;
    }
    ROS_INFO_STREAM("part localized: " << srv.response);

    geometry_msgs::Pose move_target = flipPose(srv.response.pose);

    // Plan for robot to move to part
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    move_group.setPoseTarget(move_target);
    move_group.move();

    // Plan cartesian path
    myworkcell_core::PlanCartesianPath cartesian_srv;
    cartesian_srv.request.pose = move_target;
    if (!cartesian_client_.call(cartesian_srv))
    {
      ROS_ERROR("Could not plan for path");
      return;
    }

    // Execute descartes-planned path directly (bypassing MoveIt)
    ROS_INFO("Got cart path, executing");
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = cartesian_srv.response.trajectory;
    ac_.sendGoal(goal);
    ac_.waitForResult();
    ROS_INFO("Done");
  }
  /**
   * @brief flipPose rotates the input transform by 180 degrees about the
   * x-axis
   * @param in geometry_msgs::Pose reference to the input transform
   * @return geometry_msgs::Pose of the flipped output transform
   */
  geometry_msgs::Pose flipPose(const geometry_msgs::Pose& in) const
  {
    tf::Transform in_tf;
    tf::poseMsgToTF(in, in_tf);
    tf::Quaternion flip_rot(tf::Vector3(1, 0, 0), 0);
    tf::Transform flipped = in_tf * tf::Transform(flip_rot);
    geometry_msgs::Pose out;
    tf::poseTFToMsg(flipped, out);
    return out;
  }

private:
  // Planning components
  ros::ServiceClient vision_client_;
  ros::ServiceClient cartesian_client_;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_;
};

/**
 * @brief main is the ros interface for the ScanNPlan Class
 * @param argc ROS uses this to parse remapping arguments from the command line.
 * @param argv ROS uses this to parse remapping arguments from the command line.
 * @return ROS provides typical return codes, 0 or -1, depending on the
 * execution.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "myworkcell_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_node_handle("~");
  ros::AsyncSpinner async_spinner (1);

  ROS_INFO("ScanNPlan node has been initialized");

  std::string base_frame;
  private_node_handle.param<std::string>("base_frame", base_frame, "world"); // parameter name, string object reference, default value

  ScanNPlan app(nh);
  ros::Duration(.5).sleep();  // wait for the class to initialize

  async_spinner.start();
  app.start(base_frame);

  ros::waitForShutdown();
}
