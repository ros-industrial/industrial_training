#include <ros/ros.h>
#include <Python.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h> //hydro
#include <lesson_perception/FilterCloud.h>
#include <lesson_perception/PyCloud.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h> //hydro
#include <pcl_ros/transforms.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>

/* ========================================
 * Fill Code: Filter Parameters
 * ========================================*/
double leaf_size_;
double passThrough_max_;
double passThrough_min_;
double maxIterations_;
double distThreshold_;
double clustTol_;
double clustMax_;
double clustMin_;

ros::NodeHandlePtr nh_;
std::string static_link_name;


/* ========================================
 * Fill Code: VOXEL GRID
 * ========================================*/
pcl::PointCloud<pcl::PointXYZ>::Ptr voxelGrid(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double leaf_size)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_filtered (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(cloud);
  voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_filter.filter(*cloud_voxel_filtered);
  return cloud_voxel_filtered;
}
/* ========================================
 * Fill Code: PASSTHROUGH FILTER(S)
 * ========================================*/
pcl::PointCloud<pcl::PointXYZ>::Ptr passThrough(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_voxel_filtered)
{
  //allocate an empty pointer
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyzf_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  //filter x
  pcl::PassThrough<pcl::PointXYZ> pass_x;
  pass_x.setInputCloud(cloud_voxel_filtered);
  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits(-0.50,0.50);
  pass_x.filter(*xyzf_cloud);
  //filter y
  pcl::PassThrough<pcl::PointXYZ> pass_y;
  pass_y.setInputCloud(xyzf_cloud);
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits(-0.50, 0.50);
  pass_y.filter(*xyzf_cloud);
  //filter z
  pcl::PassThrough<pcl::PointXYZ> pass_z;
  pass_z.setInputCloud(xyzf_cloud);
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(-0.50, 0.50);
  pass_z.filter(*xyzf_cloud);
  return xyzf_cloud;
}

  /* ========================================
   * Fill Code: CROPBOX (OPTIONAL)
   * ========================================*/

/* ========================================
 * Fill Code: PLANE SEGEMENTATION
 * ========================================*/
pcl::PointCloud<pcl::PointXYZ>::Ptr planeSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr (&input_cloud))
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud (new pcl::PointCloud<pcl::PointXYZ>(*input_cloud));
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
  seg.setMaxIterations (200);
  seg.setDistanceThreshold (0.01); //keep as 1cm
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

  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud_f);
  if (cloud_f->points.size () == 0)
  {
    ROS_WARN_STREAM ("Could not estimate a planar model for the given dataset.") ;

    //break;
  }
  else
  {
    ROS_INFO_STREAM("PointCloud representing the planar component: " << cloud_f->points.size () << " data points." );
  }
  return cloud_f;
}

  /* ========================================
   * Fill Code: EUCLIDEAN CLUSTER EXTRACTION
   * ========================================*/
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
clusterExtraction(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &input_cloud)
{
  ROS_INFO_STREAM("cluster input size: " << input_cloud->size());
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
  tree->setInputCloud (input_cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.01); // 2cm
  ec.setMinClusterSize (25);
  ec.setMaxClusterSize (10000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (input_cloud);
  ec.extract (cluster_indices);

  ROS_INFO_STREAM("cluster size: " << cluster_indices.size());

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud_cluster->points.push_back(input_cloud->points[*pit]);
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    std::cout << "Cluster has " << cloud_cluster->points.size() << " points.\n";
    clusters.push_back(cloud_cluster);
  }
  return clusters;
}

  /* ========================================
   * Fill Code: PUBLISH PLANE MARKER (OPTIONAL)
   * ========================================*/

  /* ========================================
   * Fill Code: STATISTICAL OUTLIER REMOVAL (OPTIONAL)
   * ========================================*/

/* ========================================
 * Fill Code: SERVICE API
 * ========================================*/
bool filterCallback(lesson_perception::FilterCloud::Request& request,
                    lesson_perception::FilterCloud::Response& response)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (request.pcdfilename.empty())
  {
    pcl::fromROSMsg(request.input_cloud, *cloud);
    ROS_INFO_STREAM("cloud size: " <<cloud->size());
    if (cloud->empty())
    {
      ROS_ERROR("input cloud empty");
      response.success = false;
      return false;
    }
    //response.output_cloud.header=request.input_cloud.header;
  }
  else
  {
    pcl::io::loadPCDFile(request.pcdfilename, *cloud);
    //response.output_cloud.header.frame_id="kinect_link";
  }

  switch (request.operation)
  {

    case lesson_perception::FilterCloud::Request::VOXELGRID :
    {
      filtered_cloud = voxelGrid(cloud, 0.01);
      ROS_ERROR("clusterExtraction_voxelgrid");
      break;
    }

    case lesson_perception::FilterCloud::Request::PASSTHROUGH :
    {
      filtered_cloud = passThrough(cloud);
      ROS_ERROR("clusterExtraction_passThrough");
      break;
    }
    case lesson_perception::FilterCloud::Request::PLANESEGMENTATION :
    {
      filtered_cloud = planeSegmentation(cloud);
      ROS_ERROR("clusterExtraction_planeSegmentation");
      break;
    }
    case lesson_perception::FilterCloud::Request::CLUSTEREXTRACTION :
    {
      std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> temp =clusterExtraction(cloud);
      ROS_INFO_STREAM("clusterextraction size: "<<temp[0]->points.size());
      if (temp.size()>0)
      {
        filtered_cloud = temp[0];
      }
      //filtered_cloud = clusterExtraction(cloud)[0];
      ROS_ERROR("clusterExtraction_clusterExtraction");
      break;
    }
    default :
    {
      ROS_ERROR("no point cloud found");
      return false;
    }

   }

/*
 * SETUP PUBLISHERS
 */
  pcl::toROSMsg(*filtered_cloud, response.output_cloud);
  response.output_cloud.header=request.input_cloud.header;
  response.output_cloud.header.frame_id="kinect_link";
//  ros::Publisher pub;
//  pub = nh_->advertise<sensor_msgs::PointCloud2>(request.topic, 10, true);
//  pub.publish(response.output_cloud);
  response.success = true;
  return true;
}

  /* ========================================
   * Fill Code: PUBLISH OTHER MARKERS (OPTIONAL)
   * ========================================*/


  /* ========================================
  * BROADCAST TRANSFORM (OPTIONAL)
  * ========================================*/


  /* ========================================
   * Fill Code: POLYGONAL SEGMENTATION (OPTIONAL)
   * ========================================*/

int main(int argc, char *argv[])
{
  /*
   * INITIALIZE ROS NODE
   */
  ros::init(argc, argv, "perception_node");
  ros::NodeHandle priv_nh_("~");

  priv_nh_.param<double>("leaf_size", leaf_size_, 0.0f);
  priv_nh_.param<double>("passThrough_max", passThrough_max_, 1.0f);
  priv_nh_.param<double>("passThrough_min", passThrough_min_, -1.0f);
  priv_nh_.param<double>("maxIterations", maxIterations_, 200.0f);
  priv_nh_.param<double>("distThreshold", distThreshold_, 0.01f);
  priv_nh_.param<double>("clustTol", clustTol_, 0.01f);
  priv_nh_.param<double>("clustMax", clustMax_, 10000.0);
  priv_nh_.param<double>("clustMin", clustMin_, 300.0f);

//  if (argc != 8)
//  {
//    ROS_ERROR("Invalid number of parameters\nusage: perception_node child_frame_name x y z roll pitch yaw");
//  }
//  static_link_name = argv[1];
//  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
//  geometry_msgs::TransformStamped static_transformStamped;

//  static_transformStamped.header.stamp = ros::Time::now();
//  static_transformStamped.header.frame_id = "world_frame";
//  static_transformStamped.child_frame_id = static_link_name;
//  static_transformStamped.transform.translation.x = atof(argv[2]);
//  static_transformStamped.transform.translation.y = atof(argv[3]);
//  static_transformStamped.transform.translation.z = atof(argv[4]);
//  tf2::Quaternion quat;
//  quat.setRPY(atof(argv[5]), atof(argv[6]), atof(argv[7]));
//  static_transformStamped.transform.rotation.x = quat.x();
//  static_transformStamped.transform.rotation.y = quat.y();
//  static_transformStamped.transform.rotation.z = quat.z();
//  static_transformStamped.transform.rotation.w = quat.w();
//  static_broadcaster.sendTransform(static_transformStamped);
//  ROS_INFO("Spinning until killed publishing %s to world", static_link_name.c_str());

  nh_.reset(new ros::NodeHandle());
  ros::ServiceServer server = nh_->advertiseService("filter_cloud", &filterCallback);
  ros::spin();
  return 0;
}
