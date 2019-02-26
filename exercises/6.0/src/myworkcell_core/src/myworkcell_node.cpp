#include <ros/ros.h>
#include <myworkcell_core/LocalizePart.h>
#include <tf/tf.h>
#include <moveit/move_group_interface/move_group_interface.h>

/**
* @brief The ScanNPlan class is a client of the vision and path plan servers.  The ScanNPLan class takes
* these services, computes transforms and published commands to the robot.
*/
class ScanNPlan
{
public:
  ScanNPlan(ros::NodeHandle& nh)
  {
    vision_client_ = nh.serviceClient<myworkcell_core::LocalizePart>("localize_part");
  }

  /**
   * @brief start performs the motion planning and execution functions of the ScanNPlan of
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

    geometry_msgs::Pose move_target = srv.response.pose;
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");

    // Plan for robot to move to part
    move_group.setPoseReferenceFrame(base_frame);
    move_group.setPoseTarget(move_target);
    move_group.move();
  }

private:
  // Planning components
  ros::ServiceClient vision_client_;
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
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();

  ROS_INFO("ScanNPlan node has been initialized");

  std::string base_frame;
  private_node_handle.param<std::string>("base_frame", base_frame, "world"); // parameter name, string object reference, default value

  ScanNPlan app(nh);

  ros::Duration(.5).sleep();  // wait for the class to initialize
  app.start(base_frame);

  ros::waitForShutdown();
}
