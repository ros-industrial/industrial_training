#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Geometry>
#include <tf_conversions/tf_eigen.h>

int main(int argc, char** argv)
{
  // Define ROS info
  ros::init(argc, argv, "extrinsic_calibration");
  ros::NodeHandle nh;
  static tf::TransformBroadcaster camera_broadcaster;
  static tf::TransformBroadcaster target_broadcaster;
  static tf::TransformListener listener;

  // Get ROS params
  bool sim_robot;
  double targ_x, targ_y, targ_z;
  std::string camera_link;
  nh.param<bool>("calibration_node/sim_robot", sim_robot, true);
  nh.param<double>("calibration_node/targ_x", targ_x, 0.4);
  nh.param<double>("calibration_node/targ_y", targ_y, 0.0);
  nh.param<double>("calibration_node/targ_z", targ_z, 0.0);
  nh.param<std::string>("calibration_node/camera_link", camera_link, "camera_link");

  // Update location of AR tag
  tf::Transform targ_transform;
  tf::Quaternion targ_q;
  if (!sim_robot)
  {
    ROS_INFO("Move robot to target to calibration location");
  }
  else{
    targ_q.setRPY(0, 0, 0);
    targ_transform.setOrigin(tf::Vector3(targ_x, targ_y, targ_z));
    targ_transform.setRotation(targ_q);
  }
  ROS_INFO("Sending target transform");
  target_broadcaster.sendTransform(
        tf::StampedTransform(targ_transform, ros::Time::now(), "base_link", "target_link"));
  ros::Duration(1.0).sleep();

  // Broadcast initial camera transform that is based on target_link
  tf::Transform transform;
  tf::Quaternion q;
  q.setRPY(0,0,0);
  transform.setOrigin(tf::Vector3(0,0,0));
  transform.setRotation(q);
  camera_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "target_link", camera_link));
  tf::StampedTransform stamped;
  ROS_INFO("Sending initial camera transform");
  ros::Duration(1.0).sleep();

  // Calibrate the camera position
  bool calibrating = true;

  while (calibrating)
  {
   int iters = 20;
   double rx = 0;
   double ry = 0;
   double rz = 0;
   double rw = 0;
   double x = 0;
   double y = 0;
   double z = 0;
    for (int ind = 0; ind < iters; ind++)
    {
      target_broadcaster.sendTransform(
          tf::StampedTransform(targ_transform, ros::Time::now(), "base_link", "target_link"));
      try
      {
        // Get actual location of target in camera frame
        listener.lookupTransform("/ar_marker_0", camera_link, ros::Time(0), stamped);
        // Update location of camera based on that transform
        stamped.frame_id_ = "target_link";
        stamped.child_frame_id_ = camera_link;
        camera_broadcaster.sendTransform(stamped);

        // Take average
        // Note: There are better ways to average quaternions (https://stackoverflow.com/questions/12374087/average-of-multiple-quaternions)
        rx += stamped.getRotation().x()/float(iters);
        ry += stamped.getRotation().y()/float(iters);
        rz += stamped.getRotation().z()/float(iters);
        rw += stamped.getRotation().w()/float(iters);
        x += stamped.getOrigin().x()/float(iters);
        y += stamped.getOrigin().y()/float(iters);
        z += stamped.getOrigin().z()/float(iters);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
      }

      ros::spinOnce();
      ros::Duration(0.25).sleep();
    }

    // Update location of camera based on the average transform
    stamped.frame_id_ = "target_link";
    stamped.child_frame_id_ = camera_link;
    stamped.setRotation(tf::Quaternion(rx,ry,rz,rw).normalize());
    stamped.setOrigin(tf::Vector3(x,y,z));
    camera_broadcaster.sendTransform(stamped);

    // Get transform from robot base to the camera and display it for the user to copy.
    // Ideally, this would automatically be written to a file in a real system
    listener.lookupTransform("/base_link", camera_link, ros::Time(0), stamped);
    double yaw, pitch, roll;
    stamped.getBasis().getRPY(roll, pitch, yaw);
    ROS_ERROR("Calibrated camera location in base_link frame. Update these values in launch file");
    std::cout << "Link name: " << camera_link << '\n';
    std::cout << stamped.getOrigin()[0] << " " << stamped.getOrigin()[1] << " "
              << stamped.getOrigin()[2] << " " << roll << " "
              << pitch << " " << yaw << "\n";

    ROS_ERROR(" Exit? y/n");
    char input;
    std::cin >> input;
    if (input == 'y')
    {
      calibrating = false;
    }
  }
}
