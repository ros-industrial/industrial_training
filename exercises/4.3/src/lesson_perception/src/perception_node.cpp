#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h> //hydro

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h> //hydro
#include "pcl_ros/transforms.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <visualization_msgs/Marker.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>

//polygonal segmentation
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/surface/convex_hull.h>

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
  double voxel_leaf_size;
  double x_filter_min, x_filter_max, y_filter_min, y_filter_max, z_filter_min, z_filter_max;
  double plane_max_iter, plane_dist_thresh;
  double cluster_tol;
  int cluster_min_size, cluster_max_size;
  /*world_frame="world_frame";
  camera_frame="kinect_link";
  cloud_topic="camera/depth_registered/points";
  voxel_leaf_size=0.001f;
  x_filter_min=-2.5;
  x_filter_max=2.5;
  y_filter_min=-2.5;
  y_filter_max=2.5;
  z_filter_min=-2.5;
  z_filter_max=1.0;
  plane_max_iter=50;
  plane_dist_thresh=0.05;
  cluster_tol=0.02;
  cluster_min_size=100;
  cluster_max_size=50000;*/
  priv_nh_.getParam("cloud_topic", cloud_topic);
  priv_nh_.getParam("world_frame", world_frame);
  priv_nh_.getParam("camera_frame", camera_frame);
  priv_nh_.getParam("voxel_leaf_size", voxel_leaf_size);
  priv_nh_.getParam("x_filter_min", x_filter_min);
  priv_nh_.getParam("x_filter_max", x_filter_max);
  priv_nh_.getParam("y_filter_min", y_filter_min);
  priv_nh_.getParam("y_filter_max", y_filter_max);
  priv_nh_.getParam("z_filter_min", z_filter_min);
  priv_nh_.getParam("z_filter_max", z_filter_max);
  priv_nh_.getParamCached("plane_max_iterations", plane_max_iter);
  priv_nh_.getParamCached("plane_distance_threshold", plane_dist_thresh);
  priv_nh_.getParam("cluster_tolerance", cluster_tol);
  priv_nh_.getParam("cluster_min_size", cluster_min_size);
  priv_nh_.getParam("cluster_max_size", cluster_max_size);

  /*
   * SETUP PUBLISHERS
   */
  ros::Publisher object_pub, cluster_pub, pose_pub, vis_pub, scene_wo_box;
  object_pub = nh.advertise<sensor_msgs::PointCloud2>("object_cluster", 1);
  cluster_pub = nh.advertise<sensor_msgs::PointCloud2>("primary_cluster", 1);
  scene_wo_box = nh.advertise<sensor_msgs::PointCloud2>("scene_minus_box", 1);
  vis_pub  = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 1);

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


  //MAKE TIMERS FOR PROCESS (OPTIONAL)
  ros::Time start_init = ros::Time::now();

  /* ========================================
   * Fill Code: VOXEL GRID
   * ========================================*/
  //input clout must be a pointer, so we make a new cloud_ptr from the cloud object
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>(cloud));
  //output cloud - set up as pointer to ease transition into further processing
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_filtered (new pcl::PointCloud<pcl::PointXYZ> ());
  //create an instance of the pcl VoxelGrid
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud (cloud_ptr);
  voxel_filter.setLeafSize (float(voxel_leaf_size), float(voxel_leaf_size), float(voxel_leaf_size));
  voxel_filter.filter (*cloud_voxel_filtered);

  ROS_INFO_STREAM("Original cloud  had " << cloud_voxel_filtered->size() << " points");
  //ROS_INFO_STREAM("Downsampled cloud  with " << cloud_voxel_filtered->size() << " points");

  /* ========================================
   * Fill Code: PASSTHROUGH FILTER(S)
   * ======================*/
 //step 1- filter in x
  pcl::PointCloud<pcl::PointXYZ> xf_cloud, yf_cloud, zf_cloud;
  pcl::PassThrough<pcl::PointXYZ> pass_x;
  pass_x.setInputCloud(cloud_voxel_filtered);
  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits(x_filter_min,x_filter_max);
  pass_x.filter(xf_cloud);

 //pass to filter in y
  pcl::PointCloud<pcl::PointXYZ>::Ptr xf_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(xf_cloud));
  pcl::PassThrough<pcl::PointXYZ> pass_y;
  pass_y.setInputCloud(xf_cloud_ptr);
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits(y_filter_min, y_filter_max);
  pass_y.filter(yf_cloud);

  //pass to filter in z
  pcl::PointCloud<pcl::PointXYZ>::Ptr yf_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(yf_cloud));
  pcl::PassThrough<pcl::PointXYZ> pass_z;
  pass_z.setInputCloud(yf_cloud_ptr);
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(z_filter_min, z_filter_max);
  pass_z.filter(zf_cloud);

  /* ========================================
  * Fill Code: CROPBOX (OPTIONAL)
   * Instead of three passthrough filters, the cropbox filter can be used
   * The user should choose one or the other method
   * ========================================*/
  pcl::PointCloud<pcl::PointXYZ> xyz_filtered_cloud;
  pcl::CropBox<pcl::PointXYZ> crop;
  crop.setInputCloud(cloud_voxel_filtered);
  Eigen::Vector4f min_point = Eigen::Vector4f(x_filter_min, y_filter_min, z_filter_min, 0);
  Eigen::Vector4f max_point = Eigen::Vector4f(x_filter_max, y_filter_max, z_filter_max, 0);
  crop.setMin(min_point);
  crop.setMax(max_point);
  crop.filter(xyz_filtered_cloud);

  /* ========================================
   * Fill Code: PLANE SEGEMENTATION
   * ========================================*/
  pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>(zf_cloud)); //this passes in either passthrough or crop filtered cloud.
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (plane_max_iter);
  seg.setDistanceThreshold (plane_dist_thresh);
  // Segment the largest planar component from the cropped cloud
  seg.setInputCloud (cropped_cloud);
  seg.segment (*inliers, *coefficients);
  if (inliers->indices.size () == 0)
  {
    ROS_WARN_STREAM ("Could not estimate a planar model for the given dataset.") ;
    //break;
  }

  // Extract the planar inliers from the input cloud
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cropped_cloud);
  extract.setIndices(inliers);
  extract.setNegative (false);

  // Get the points associated with the planar surface
  extract.filter (*cloud_plane);
  ROS_INFO_STREAM("PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." );

  /* ========================================
   * Fill Code: PUBLISH PLANE MARKER (OPTIONAL)
   * ========================================*/
  visualization_msgs::Marker table_marker;


  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud_f);

  /* ========================================
   * Fill Code: EUCLIDEAN CLUSTER EXTRACTION (OPTIONAL/RECOMMENDED)
   * ========================================*/
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  *cloud_filtered = *cloud_f;
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (cluster_tol); // 2cm
  ec.setMinClusterSize (cluster_min_size);
  ec.setMaxClusterSize (cluster_max_size);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  std::vector<sensor_msgs::PointCloud2::Ptr> pc2_clusters;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > clusters;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud_cluster->points.push_back(cloud_filtered->points[*pit]);
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    std::cout << "Cluster has " << cloud_cluster->points.size() << " points.\n";
    clusters.push_back(cloud_cluster);
    sensor_msgs::PointCloud2::Ptr tempROSMsg(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*cloud_cluster, *tempROSMsg);
    pc2_clusters.push_back(tempROSMsg);
  }
  pc2_clusters.at(0)->header.frame_id=world_frame;
  pc2_clusters.at(0)->header.stamp=ros::Time::now();
  cluster_pub.publish(pc2_clusters.at(0));

  /* ========================================
   * Fill Code: STATISTICAL OUTLIER REMOVAL (OPTIONAL)
   * ========================================*/
  pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud_ptr= clusters.at(0);
  pcl::PointCloud<pcl::PointXYZ>::Ptr sor_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cluster_cloud_ptr);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*sor_cloud_filtered);

  //OPTIONAL TIMERS
  ros::Time finish_process = ros::Time::now();
  ros::Duration total_process = finish_process - start_init;
  ROS_INFO_STREAM("Point Cloud processing took "<< total_process<<" s");

  /* ========================================
   * CONVERT POINTCLOUD PCL->ROS
   * PUBLISH CLOUD
   * Fill Code: UPDATE AS NECESSARY
   * ========================================*/
  sensor_msgs::PointCloud2::Ptr pc2_cloud (new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*sor_cloud_filtered, *pc2_cloud);
  pc2_cloud->header.frame_id=world_frame;
  pc2_cloud->header.stamp=ros::Time::now();
  object_pub.publish(*pc2_cloud);

  /* ========================================
   * Fill Code: PUBLISH OTHER MARKERS (OPTIONAL)
   * ========================================*/


  /* ========================================
  * BROADCAST TRANSFORM (OPTIONAL)
  * ========================================*/
    static tf::TransformBroadcaster br;
    tf::Transform part_transform;
    //Here in the tf::Vector3(x,y,z) x,y, and z should be calculated based on the pointcloud filtering results
    part_transform.setOrigin( tf::Vector3(sor_cloud_filtered->at(1).x, sor_cloud_filtered->at(1).y, sor_cloud_filtered->at(1).z) );
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    part_transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(part_transform, ros::Time::now(), world_frame, "part"));

  /* ========================================
   * Fill Code: POLYGONAL SEGMENTATION (OPTIONAL)
   * ========================================*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr sensor_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>(cloud));
    pcl::PointCloud<pcl::PointXYZ>::Ptr prism_filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pick_surface_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
    pcl::ExtractIndices<pcl::PointXYZ> extract_ind;
    pcl::PointIndices::Ptr pt_inliers (new pcl::PointIndices());

    prism.setInputCloud(sensor_cloud_ptr);

    // create prism surface
    double box_length=0.25;
    double box_width=0.25;
    pick_surface_cloud_ptr->width = 5;
    pick_surface_cloud_ptr->height = 1;
    pick_surface_cloud_ptr->points.resize(5);

    pick_surface_cloud_ptr->points[0].x = 0.5f*box_length;
    pick_surface_cloud_ptr->points[0].y = 0.5f*box_width;
    pick_surface_cloud_ptr->points[0].z = 0;

    pick_surface_cloud_ptr->points[1].x = -0.5f*box_length;
    pick_surface_cloud_ptr->points[1].y = 0.5f*box_width;
    pick_surface_cloud_ptr->points[1].z = 0;

    pick_surface_cloud_ptr->points[2].x = -0.5f*box_length;
    pick_surface_cloud_ptr->points[2].y = -0.5f*box_width;
    pick_surface_cloud_ptr->points[2].z = 0;

    pick_surface_cloud_ptr->points[3].x = 0.5f*box_length;
    pick_surface_cloud_ptr->points[3].y = -0.5f*box_width;
    pick_surface_cloud_ptr->points[3].z = 0;

    pick_surface_cloud_ptr->points[4].x = 0.5f*box_length;
    pick_surface_cloud_ptr->points[4].y = 0.5f*box_width;
    pick_surface_cloud_ptr->points[4].z = 0;

    Eigen::Affine3d eigen3d;
    tf::transformTFToEigen(part_transform,eigen3d);
    pcl::transformPointCloud(*pick_surface_cloud_ptr,*pick_surface_cloud_ptr,Eigen::Affine3f(eigen3d));

    prism.setInputPlanarHull( pick_surface_cloud_ptr);
    prism.setHeightLimits(-10,10);
    prism.segment(*pt_inliers);

    //extracting remaining points
    extract_ind.setInputCloud(sensor_cloud_ptr);
    extract_ind.setIndices(pt_inliers);
    extract_ind.setNegative(true);
    extract_ind.filter(*prism_filtered_cloud);;

    //convert and publish the cloud without box
    sensor_msgs::PointCloud2::Ptr scene_minus_box_cloud (new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*prism_filtered_cloud, *scene_minus_box_cloud); //into ros, from
    scene_minus_box_cloud->header.frame_id=world_frame;
    scene_minus_box_cloud->header.stamp=ros::Time::now();
    scene_wo_box.publish(scene_minus_box_cloud);


  }
  return 0;
}
