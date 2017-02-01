#include <ros/ros.h>
#include <myworkcell_core/LocalizePart.h>
#include <myworkcell_core/PlanCartesianPath.h>
#include <moveit/move_group_interface/move_group.h>

class ScanNPlan
{
public:
  ScanNPlan(ros::NodeHandle& nh_in) : group_("manipulator")
  {
    group_.setPlannerId("RRTConnectkConfigDefault");
    nh = nh_in;
    vision_client_ = nh.serviceClient<myworkcell_core::LocalizePart>("localize_part");
    cartesian_client_ = nh.serviceClient<myworkcell_core::PlanCartesianPath>("plan_path");
    pub_ = nh.advertise<trajectory_msgs::JointTrajectory>("/joint_path_command", 1);

    nh.param<std::string>("ref_frame_param", ref_frame, "world");
  }

geometry_msgs::Pose transformPose(const geometry_msgs::Pose& in) const
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

  void start()
  {
    ROS_INFO("Attempting to localize part");
    // Localize the part
    myworkcell_core::LocalizePart srv;
    if (nh.getParam("ref_frame_param", ref_frame));
    srv.request.base_frame = ref_frame;

    if (!vision_client_.call(srv))
    {
      ROS_ERROR("Could not localize part");
      return;
    }
    ROS_INFO_STREAM("part localized: " << srv.response);


    srv.response.pose = transformPose(srv.response.pose);

    // Plan for robot to move to part    
    group_.setPoseTarget(srv.response.pose);
    group_.move();
    ROS_INFO("Done moving, planning cart path");

    // Now let's plan a motion to 
    // Plan cartesian path
    myworkcell_core::PlanCartesianPath cartesian_srv;
    cartesian_srv.request.pose = srv.response.pose;
    if (!cartesian_client_.call(cartesian_srv))
    {
      ROS_ERROR("Could not plan for path");
      return;
    }

    //     Execute path
    ROS_INFO("Got cart path, executing");
    pub_.publish(cartesian_srv.response.trajectory);
    ROS_INFO("Done");
  }

private:
  // Planning components
  ros::ServiceClient vision_client_;
  ros::ServiceClient cartesian_client_;
  ros::Publisher pub_;

  std::string ref_frame;
  ros::NodeHandle nh;
  moveit::planning_interface::MoveGroup group_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "myworkcell_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner async_spinner (1);

  // Hello World
  ROS_INFO("Hello, World from a ROS Node");

  ScanNPlan app (nh);

  ros::Duration(.5).sleep();

  async_spinner.start();
  app.start();

  ros::spin();
}
