#include <ros/ros.h>
#include <myworkcell_core/LocalizePart.h>
#include <moveit/move_group_interface/move_group.h>

static geometry_msgs::Pose transformPose(const geometry_msgs::Pose& in)
{
  tf::Transform in_world;
  tf::poseMsgToTF(in, in_world);

  tf::Quaternion flip_z (tf::Vector3(1, 0, 0), M_PI);
  tf::Transform flip (flip_z);

  in_world = in_world * flip;

  geometry_msgs::Pose msg;
  tf::poseTFToMsg(in_world, msg);
  return msg;
}

class ScanNPlan
{
public:
  ScanNPlan(ros::NodeHandle& nh) : group_("manipulator")
  {
    vision_client_ = nh.serviceClient<myworkcell_core::LocalizePart>("localize_part");
  }

  void start(const std::string& base_frame)
  {
    ROS_INFO("Attempting to localize part");
    // Localize the part
    myworkcell_core::LocalizePart srv;
    srv.request.base_frame = base_frame;
    if (!vision_client_.call(srv))
    {
      ROS_ERROR("Could not localize part");
      return;
    }
    ROS_INFO_STREAM("part localized: " << srv.response);

    // Plan for robot to move to part    
    group_.setPoseTarget(srv.response.pose);
    group_.move();
    ROS_INFO("Done moving, planning cart path");
  }

private:
  // Planning components
  ros::ServiceClient vision_client_;
  moveit::planning_interface::MoveGroup group_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "myworkcell_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh ("~");
  ros::AsyncSpinner async_spinner (1);

  // Hello World
  ROS_INFO("Hello, World from a ROS Node");

  std::string base_frame;
  pnh.param<std::string>("base_frame", base_frame, "world");
  
  ROS_INFO_STREAM("Using user base_frame: " << base_frame);
  ScanNPlan app (nh);

  // Give time for vision node to start
  // Prefer something like service::waitForExist() instead
  ros::Duration(.5).sleep();

  async_spinner.start();
  app.start(base_frame);

  ros::spin();
}
