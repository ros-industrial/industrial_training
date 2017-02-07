#include <ros/ros.h>
#include <myworkcell_core/LocalizePart.h>

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
    if (!vision_client_.call(srv))
    {
      ROS_ERROR("Could not localize part");
      return;
    }
    ROS_INFO_STREAM("part localized: " << srv.response);
  }

private:
  // Planning components
  ros::ServiceClient vision_client_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "myworkcell_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh ("~");
  ros::AsyncSpinner async_spinner (1);

  // Hello World
  ROS_INFO("Hello, World from a ROS Node");

  // Load parameters
  std::string base_frame; // for localizing the part
  pnh.param<std::string>("base_frame", base_frame, "world");

  ScanNPlan app (nh);

  ros::Duration(.5).sleep();

  async_spinner.start();
  app.start(base_frame);

  ros::spin();
}
