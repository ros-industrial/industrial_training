#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h> //hydro

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h> //hydro
#include "pcl_ros/transforms.h"

//#include <pcl/filters/voxel_grid.h>
//#include <pcl/filters/passthrough.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/segmentation/extract_clusters.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/filters/statistical_outlier_removal.h>

int main(int argc, char *argv[])
{
  /*
   * INITIALIZE ROS NODE
   */
  ros::init(argc, argv, "perception_node");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh_("~");

  /*
   * SET UP PARAMETERS (COULD TO BE INPUT FROM LAUNCH FILE/TERMINAL)
   */
  std::string cloud_topic, world_frame, camera_frame;
  world_frame="world_frame";
  camera_frame="kinect_link";
  cloud_topic="kinect/depth_registered/points";

  /*
   * SETUP PUBLISHERS
   */
  ros::Publisher object_pub, cluster_pub, pose_pub;
  object_pub = nh.advertise<sensor_msgs::PointCloud2>("object_cluster", 1);
  cluster_pub = nh.advertise<sensor_msgs::PointCloud2>("primary_cluster", 1);

 while (ros::ok())
 {
  /*
   * LISTEN FOR POINTCLOUD
   */
  std::string topic = nh.resolveName(cloud_topic);
  ROS_INFO_STREAM("Cloud service called; waiting for a PointCloud2 on topic "<< topic);
  sensor_msgs::PointCloud2::ConstPtr recent_cloud =
               ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh);

  /*
   * TRANSFORM POINTCLOUD FROM CAMERA FRAME TO WORLD FRAME
   */
  tf::TransformListener listener;
  tf::StampedTransform stransform;
  try
  {
    listener.waitForTransform(world_frame, camera_frame,  ros::Time::now(), ros::Duration(6.0));
    listener.lookupTransform(world_frame, camera_frame,  ros::Time::now(), stransform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
  }
  sensor_msgs::PointCloud2 transformed_cloud;
  pcl_ros::transformPointCloud(world_frame, *recent_cloud, transformed_cloud, listener);

  /*
   * CONVERT POINTCLOUD ROS->PCL
   */
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (transformed_cloud, cloud);

  /*
   * VOXEL GRID
   */
  //input clout must be a pointer, so we make a new cloud_ptr from the cloud object
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(cloud));
  //output cloud - set up as pointer to ease transition into further processing
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_filtered (new pcl::PointCloud<pcl::PointXYZ> ());
  //create an instance of the pcl VoxelGrid
  
  ROS_INFO_STREAM("Original cloud  had " << cloud_ptr->size() << " points");
  ROS_INFO_STREAM("Downsampled cloud  with " << cloud_voxel_filtered->size() << " points");

  /*
   * PASSTHROUGH FILTER(S)
   */
 

  /*
   * PLANE SEGEMENTATION
   */
 
  /*
   * PUBLISH PLANE MARKER (OPTIONAL)
   */


  /*
   * EUCLIDEAN CLUSTER EXTRACTION (OPTIONAL)
   */


  /*
   * STATISTICAL OUTLIER REMOVAL (OPTIONAL)
   */


  /*
   * PUBLISH CLOUD
   */
  sensor_msgs::PointCloud2::Ptr pc2_cloud (new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*cloud_ptr, *pc2_cloud);
  pc2_cloud->header.frame_id=world_frame;
  pc2_cloud->header.stamp=ros::Time::now();
  object_pub.publish(pc2_cloud);

  /*
   * PUBLISH OTHER MARKERS (OPTIONAL)
   */

  /*
   * BROADCAST TRANSFORM
   */
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  //Here in the tf::Vector3(x,y,z) x,y, and z should be calculated based on the pointcloud filtering results
  transform.setOrigin( tf::Vector3(cloud_ptr->at(1).x, cloud_ptr->at(1).y, cloud_ptr->at(1).z) );
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), world_frame, "part"));

  }
  return 0;
}
