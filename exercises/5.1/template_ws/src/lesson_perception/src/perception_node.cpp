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
//#include <pcl/filters/crop_box.h>
//#include <pcl/filters/statistical_outlier_removal.h>
//#include <tf_conversions/tf_eigen.h>
//#include <pcl/segmentation/extract_polygonal_prism_data.h>

int main(int argc, char *argv[])
{
  /*
   * INITIALIZE ROS NODE
   */
  ros::init(argc, argv, "perception_node");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh_("~");

  /*
   * SET UP PARAMETERS (COULD BE INPUT FROM LAUNCH FILE/TERMINAL)
   */
  std::string cloud_topic, world_frame, camera_frame;
  world_frame="kinect_link";
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
    listener.waitForTransform(world_frame, recent_cloud->header.frame_id,  ros::Time::now(), ros::Duration(6.0));
    listener.lookupTransform(world_frame, recent_cloud->header.frame_id,  ros::Time(0), stransform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
  }
  sensor_msgs::PointCloud2 transformed_cloud;
//  sensor_msgs::PointCloud2::ConstPtr recent_cloud =
//               ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh);
  pcl_ros::transformPointCloud(world_frame, stransform, *recent_cloud, transformed_cloud);

  /*
   * CONVERT POINTCLOUD ROS->PCL
   */
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (transformed_cloud, cloud);

  /* ========================================
   * Fill Code: VOXEL GRID
   * ========================================*/


  //ROS_INFO_STREAM("Original cloud  had " << cloud_ptr->size() << " points");
  //ROS_INFO_STREAM("Downsampled cloud  with " << cloud_voxel_filtered->size() << " points");

  /* ========================================
   * Fill Code: PASSTHROUGH FILTER(S)
   * ========================================*/
  //filter in x

  //filter in y

  //filter in z 

  /* ========================================
   * Fill Code: CROPBOX (OPTIONAL)
   * ========================================*/

  /* ========================================
   * Fill Code: PLANE SEGEMENTATION
   * ========================================*/
 

  /* ========================================
   * Fill Code: PUBLISH PLANE MARKER (OPTIONAL)
   * ========================================*/


  /* ========================================
   * Fill Code: EUCLIDEAN CLUSTER EXTRACTION (OPTIONAL/RECOMMENDED)
   * ========================================*/


  /* ========================================
   * Fill Code: STATISTICAL OUTLIER REMOVAL (OPTIONAL)
   * ========================================*/


  /* ========================================
   * Fill Code: PUBLISH OTHER MARKERS (OPTIONAL)
   * ========================================*/


  /* ========================================
  * BROADCAST TRANSFORM (OPTIONAL)
  * ========================================*/


  /* ========================================
   * Fill Code: POLYGONAL SEGMENTATION (OPTIONAL)
   * ========================================*/


  /* ========================================
   * CONVERT POINTCLOUD PCL->ROS
   * PUBLISH CLOUD
   * Fill Code: UPDATE AS NECESSARY
   * ========================================*/
  sensor_msgs::PointCloud2::Ptr pc2_cloud (new sensor_msgs::PointCloud2);
  //pcl::toROSMsg(*cloud_ptr, *pc2_cloud);
  pc2_cloud->header.frame_id=world_frame;
  pc2_cloud->header.stamp=ros::Time::now();
  object_pub.publish(pc2_cloud);
  }
  return 0;
}
