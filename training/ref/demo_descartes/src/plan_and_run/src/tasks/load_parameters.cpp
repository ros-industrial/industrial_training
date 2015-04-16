
#include <plan_and_run/demo_application.h>

namespace plan_and_run
{

void DemoApplication::loadParameters()
{
  ros::NodeHandle ph("~");
  ros::NodeHandle nh;

  if(ph.getParam("group_name",config_.group_name) &&
      ph.getParam("tip_link",config_.tip_link) &&
      ph.getParam("base_link",config_.base_link) &&
      ph.getParam("world_frame",config_.world_frame) &&
      ph.getParam("trajectory/time_delay",config_.time_delay) &&
      ph.getParam("trajectory/foci_distance",config_.foci_distance) &&
      ph.getParam("trajectory/radius",config_.radius) &&
      ph.getParam("trajectory/num_points",config_.num_points) &&
      ph.getParam("trajectory/num_lemniscates",config_.num_lemniscates) &&
      ph.getParam("trajectory/center",config_.center) &&
      ph.getParam("trajectory/seed_pose",config_.seed_pose) &&
      nh.getParam("controller_joint_names",config_.joint_names) )
  {
    ROS_INFO_STREAM("Loaded application parameters");

  }
  else
  {
    ROS_ERROR_STREAM("Failed to load application parameters");
    exit(-1);
  }

}

}
