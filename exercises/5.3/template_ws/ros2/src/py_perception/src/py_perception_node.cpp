#include "py_perception/srv/filter_cloud.hpp"
#include "rclcpp/rclcpp.hpp"
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/qos.hpp>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/transforms.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include "tf2/LinearMath/Quaternion.h"
#include <pcl/segmentation/extract_polygonal_prism_data.h>

class PerceptionNode : public rclcpp::Node
{
public:
    PerceptionNode()
        : Node("perception_node")
    {
        /* ========================================
         * Fill Code: Filter Parameters
         * ========================================*/
        voxel_leaf_size = 0.02;
        x_filter_min = -2.5;
        x_filter_max = 2.5;
        y_filter_min = -2.5;
        y_filter_max = 2.5;
        z_filter_min = -2.5;
        z_filter_max = 2.5;
        plane_max_iter = 100;
        plane_dist_thresh = 0.03;
        cluster_tol = 0.02;
        cluster_min_size = 1;
        cluster_max_size = 10000;

        /*
         * SETUP SERVICE SERVER
         */
        filter_server_ =
            this->create_service<py_perception::srv::FilterCloud>("filter_cloud",
                std::bind(&PerceptionNode::filterCallback, this, std::placeholders::_1, std::placeholders::_2));
    }

        /* ========================================
         * Fill Code: VOXEL GRID
         * ========================================*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxelGrid(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double leaf_size)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>());
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
        pcl::PointCloud<pcl::PointXYZ>::Ptr xyzf_cloud(new pcl::PointCloud<pcl::PointXYZ>());

        pcl::PassThrough<pcl::PointXYZ> pass_x;
        pass_x.setInputCloud(cloud_voxel_filtered);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(x_filter_min, x_filter_max);
        pass_x.filter(*xyzf_cloud);

        pcl::PassThrough<pcl::PointXYZ> pass_y;
        pass_y.setInputCloud(xyzf_cloud);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(y_filter_min, y_filter_max);
        pass_y.filter(*xyzf_cloud);

        pcl::PassThrough<pcl::PointXYZ> pass_z;
        pass_z.setInputCloud(xyzf_cloud);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(z_filter_min, z_filter_max);
        pass_z.filter(*xyzf_cloud);

        return xyzf_cloud;
    }

        /* ========================================
         * Fill Code: PLANE SEGEMENTATION
         * ========================================*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr planeSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr (&input_cloud))
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>(*input_cloud));
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
            RCLCPP_WARN(this->get_logger(), "Could not estimate a planar model for the given dataset.") ;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cropped_cloud);
        extract.setIndices(inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);
        RCLCPP_INFO(this->get_logger(),
                    "PointCloud2 representing the planar component: '%lu' data points.", cloud_plane->points.size());

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);

        return cloud_f;
    }

        /* ========================================
         * Fill Code: EUCLIDEAN CLUSTER EXTRACTION
         * ========================================*/
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusterExtraction(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud)
    {
        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (input_cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (cluster_tol);
        ec.setMinClusterSize (cluster_min_size);
        ec.setMaxClusterSize (cluster_max_size);
        ec.setSearchMethod (tree);
        ec.setInputCloud (input_cloud);
        ec.extract (cluster_indices);

        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;

        int j = 0;
        for (const auto& cluster : cluster_indices)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

            for (const auto& idx : cluster.indices) {
                cloud_cluster->points.push_back((*input_cloud)[idx]);
            }

            cloud_cluster->width = cloud_cluster->points.size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            RCLCPP_INFO(this->get_logger(), "Cluster has '%lu' points", cloud_cluster->points.size());
            clusters.push_back(cloud_cluster);

            j++;
        }
        return clusters;
    }


        /* ========================================
         * Fill Code: STATISTICAL OUTLIER REMOVAL
         * ========================================*/


        /* ========================================
         * Fill Code: POLYGONAL SEGMENTATION
         * ========================================*/



    /* ========================================
     * Fill Code: SERVICE
     * ========================================*/
    void filterCallback(py_perception::srv::FilterCloud::Request::SharedPtr request,
                        py_perception::srv::FilterCloud::Response::SharedPtr response)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        if (request->pcdfilename.empty())
        {
            pcl::fromROSMsg(request->input_cloud, *cloud);
            RCLCPP_INFO(this->get_logger(), "cloud size: '%lu'", cloud->size());
            if (cloud->empty())
            {
                RCLCPP_ERROR(this->get_logger(), "input cloud empty");
                response->success = false;
                return;
            }
        }
        else
        {
            pcl::io::loadPCDFile(request->pcdfilename, *cloud);
        }

        switch (request->operation)
        {

            case py_perception::srv::FilterCloud::Request::VOXELGRID :
            {
                filtered_cloud = voxelGrid(cloud, 0.01);
                break;
            }

            default :
            {
                RCLCPP_ERROR(this->get_logger(), "no point cloud found");
                response->success = false;
                return;
            }

        }

        /*
         * SETUP RESPONSE
         */
        pcl::toROSMsg(*filtered_cloud, response->output_cloud);
        response->output_cloud.header = request->input_cloud.header;
        response->output_cloud.header.frame_id = "kinect_link";
        response->success = true;
    }

    /*
     * Filter Parameters
     */
    float voxel_leaf_size;
    float x_filter_min;
    float x_filter_max;
    float y_filter_min;
    float y_filter_max;
    float z_filter_min;
    float z_filter_max;
    int plane_max_iter;
    float plane_dist_thresh;
    float cluster_tol;
    int cluster_min_size;
    int cluster_max_size;

    /*
     * SERVICE SERVER
     */
    rclcpp::Service<py_perception::srv::FilterCloud>::SharedPtr filter_server_;
};


int main(int argc, char *argv[])
{
    /*
     * INITIALIZE ROS NODE
     */
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PerceptionNode>());
    rclcpp::shutdown();
    return 0;
}
