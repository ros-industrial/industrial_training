#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

int main(int argc, char** argv)
{
  // Define ROS info
  ros::init(argc, argv, "extrinsic_calibration");
  ros::NodeHandle nh;
  static tf::TransformBroadcaster camera_broadcaster;
  static tf::TransformBroadcaster target_broadcaster;
  static tf::TransformListener listener;

  // Update location of AR tag (done with robot calibration)
  tf::Transform targ_transform;
  tf::Quaternion targ_q;
  targ_q.setRPY(0, 0, 0);
  targ_transform.setOrigin(tf::Vector3(0.4, 0, 0));
  targ_transform.setRotation(targ_q);
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
  camera_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "target_link", "camera_link"));
  tf::StampedTransform stamped;
  ROS_INFO("Sending initial camera transform");
  ros::Duration(1.0).sleep();

  // Calibrate the camera position
  bool calibrating = true;
  while (calibrating)
  {
    // Ideally we would do some sort of averaging over these points
    for (int ind = 0; ind < 20; ind++)
    {
      target_broadcaster.sendTransform(
          tf::StampedTransform(targ_transform, ros::Time::now(), "base_link", "target_link"));
      try
      {
        listener.lookupTransform("/ar_marker_0", "camera_link", ros::Time(0), stamped);
        stamped.frame_id_ = "target_link";
        stamped.child_frame_id_ = "camera_link";
        camera_broadcaster.sendTransform(stamped);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
      }

      ros::spinOnce();
      ros::Duration(0.25).sleep();
    }
    listener.lookupTransform("/base_link", "camera_link", ros::Time(0), stamped);
    ROS_ERROR("Calibrated camera location in base_link frame. Update these values in launch file");
    std::cout << "Translation: [" << stamped.getOrigin()[0] << ", " << stamped.getOrigin()[1] << ", "
              << stamped.getOrigin()[2] << "]\n";
    std::cout << "Rotation: [" << stamped.getRotation()[0] << ", " << stamped.getRotation()[1] << ", "
              << stamped.getRotation()[2] << "]\n";
    ROS_ERROR(" Exit? y/n");
    char input;
    std::cin >> input;
    if (input == 'y')
    {
      calibrating = false;
    }
  }
}
