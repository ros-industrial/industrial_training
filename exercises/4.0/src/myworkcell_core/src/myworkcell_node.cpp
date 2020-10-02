#include <ros/ros.h>
#include <myworkcell_core/LocalizePart.h>
#include <moveit/move_group_interface/move_group_interface.h>

class ScanNPlan
{
public:
  ScanNPlan(ros::NodeHandle& nh)
  {
    vision_client_ = nh.serviceClient<myworkcell_core::LocalizePart>("localize_part");
  }

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
