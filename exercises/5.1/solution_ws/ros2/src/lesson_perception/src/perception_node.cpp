#include "rclcpp/rclcpp.hpp"
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/qos.hpp>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

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
#include <tf2/LinearMath/Quaternion.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

class PerceptionNode : public rclcpp::Node
{
    public:
        PerceptionNode()
        : Node("perception_node", rclcpp::NodeOptions()
                                          .allow_undeclared_parameters(true)
                                          .automatically_declare_parameters_from_overrides(true))
        {
            /*
             * SET UP PUBLISHERS
             */
            RCLCPP_INFO(this->get_logger(), "Setting up publishers");

            voxel_grid_publisher_ =
                this->create_publisher<sensor_msgs::msg::PointCloud2>("voxel_cluster", 1);
            passthrough_publisher_ =
                this->create_publisher<sensor_msgs::msg::PointCloud2>("passthrough_cluster", 1);
            plane_publisher_ =
                this->create_publisher<sensor_msgs::msg::PointCloud2>("plane_cluster", 1);
            euclidean_publisher_ =
                this->create_publisher<sensor_msgs::msg::PointCloud2>("euclidean_cluster", 1);
            stat_publisher_ =
                this->create_publisher<sensor_msgs::msg::PointCloud2>("stat_cluster", 1);
            polygon_publisher_ =
                this->create_publisher<sensor_msgs::msg::PointCloud2>("polygon_cluster", 1);


            /*
             * SET UP PARAMETERS (COULD BE INPUT FROM LAUNCH FILE/TERMINAL)
             */
            rclcpp::Parameter cloud_topic_param, world_frame_param, camera_frame_param, voxel_leaf_size_param,
                x_filter_min_param, x_filter_max_param, y_filter_min_param, y_filter_max_param, z_filter_min_param,
                z_filter_max_param, plane_max_iter_param, plane_dist_thresh_param, cluster_tol_param,
                cluster_min_size_param, cluster_max_size_param;

            RCLCPP_INFO(this->get_logger(), "Getting parameters");

            this->get_parameter_or("cloud_topic", cloud_topic_param, rclcpp::Parameter("", "/kinect/depth_registered/points"));
            this->get_parameter_or("world_frame", world_frame_param, rclcpp::Parameter("", "kinect_link"));
            this->get_parameter_or("camera_frame", camera_frame_param, rclcpp::Parameter("", "kinect_link"));
            this->get_parameter_or("voxel_leaf_size", voxel_leaf_size_param, rclcpp::Parameter("", 0.002));
            this->get_parameter_or("x_filter_min", x_filter_min_param, rclcpp::Parameter("", -2.5));
            this->get_parameter_or("x_filter_max", x_filter_max_param, rclcpp::Parameter("", 2.5));
            this->get_parameter_or("y_filter_min", y_filter_min_param, rclcpp::Parameter("", -2.5));
            this->get_parameter_or("y_filter_max", y_filter_max_param, rclcpp::Parameter("", 2.5));
            this->get_parameter_or("z_filter_min", z_filter_min_param, rclcpp::Parameter("", -2.5));
            this->get_parameter_or("z_filter_max", z_filter_max_param, rclcpp::Parameter("", 2.5));
            this->get_parameter_or("plane_max_iterations", plane_max_iter_param, rclcpp::Parameter("", 50));
            this->get_parameter_or("plane_distance_threshold", plane_dist_thresh_param, rclcpp::Parameter("", 0.05));
            this->get_parameter_or("cluster_tolerance", cluster_tol_param, rclcpp::Parameter("", 0.01));
            this->get_parameter_or("cluster_min_size", cluster_min_size_param, rclcpp::Parameter("", 1));
            this->get_parameter_or("cluster_max_size", cluster_max_size_param, rclcpp::Parameter("", 10000));

            cloud_topic = cloud_topic_param.as_string();
            world_frame = world_frame_param.as_string();
            camera_frame = camera_frame_param.as_string();
            voxel_leaf_size = float(voxel_leaf_size_param.as_double());
            x_filter_min = x_filter_min_param.as_double();
            x_filter_max = x_filter_max_param.as_double();
            y_filter_min = y_filter_min_param.as_double();
            y_filter_max = y_filter_max_param.as_double();
            z_filter_min = z_filter_min_param.as_double();
            z_filter_max = z_filter_max_param.as_double();
            plane_max_iter = plane_max_iter_param.as_int();
            plane_dist_thresh = plane_dist_thresh_param.as_double();
            cluster_tol = cluster_tol_param.as_double();
            cluster_min_size = cluster_min_size_param.as_int();
            cluster_max_size = cluster_max_size_param.as_int();

            /*
             * SET UP SUBSCRIBER
             */
            RCLCPP_INFO(this->get_logger(), "Setting up subscriber");

            cloud_subscriber_ =
                this->create_subscription<sensor_msgs::msg::PointCloud2>(
                    cloud_topic, 1, std::bind(&PerceptionNode::cloud_callback, this, std::placeholders::_1));

            /*
             * SET UP TF
             */
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            br = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }

    private:

        /*
         * LISTEN FOR PointCloud2
         */
        void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr recent_cloud)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Cloud service called; getting a PointCloud2 on topic " + cloud_topic);

            /*
             * TRANSFORM PointCloud2 FROM CAMERA FRAME TO WORLD FRAME
             */
            geometry_msgs::msg::TransformStamped stransform;

            try
            {
                stransform = tf_buffer_->lookupTransform(world_frame, recent_cloud->header.frame_id,
                                                         tf2::TimePointZero, tf2::durationFromSec(3));

            }
            catch (const tf2::TransformException & ex)
            {
                RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
            }

            sensor_msgs::msg::PointCloud2 transformed_cloud;
            pcl_ros::transformPointCloud(world_frame, stransform, *recent_cloud, transformed_cloud);

            /*
             * CONVERT PointCloud2 ROS->PCL
             */
            pcl::PointCloud<pcl::PointXYZ> cloud;
            pcl::fromROSMsg(transformed_cloud, cloud);

            /* ========================================
             * Fill Code: VOXEL GRID
             * ========================================*/
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(cloud));
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
            voxel_filter.setInputCloud(cloud_ptr);
            voxel_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
            voxel_filter.filter(*cloud_voxel_filtered);

            /* ========================================
             * Fill Code: PASSTHROUGH FILTER(S)
             * ========================================*/
            pcl::PointCloud<pcl::PointXYZ> xf_cloud, yf_cloud, zf_cloud;
            pcl::PassThrough<pcl::PointXYZ> pass_x;
            pass_x.setInputCloud(cloud_voxel_filtered);
            pass_x.setFilterFieldName("x");
            pass_x.setFilterLimits(x_filter_min, x_filter_max);
            pass_x.filter(xf_cloud);

            pcl::PointCloud<pcl::PointXYZ>::Ptr xf_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(xf_cloud));
            pcl::PassThrough<pcl::PointXYZ> pass_y;
            pass_y.setInputCloud(xf_cloud_ptr);
            pass_y.setFilterFieldName("y");
            pass_y.setFilterLimits(y_filter_min, y_filter_max);
            pass_y.filter(yf_cloud);

            pcl::PointCloud<pcl::PointXYZ>::Ptr yf_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(yf_cloud));
            pcl::PassThrough<pcl::PointXYZ> pass_z;
            pass_z.setInputCloud(yf_cloud_ptr);
            pass_z.setFilterFieldName("z");
            pass_z.setFilterLimits(z_filter_min, z_filter_max);
            pass_z.filter(zf_cloud);

            /* ========================================
             * Fill Code: CROPBOX (Optional)
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
            pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>(xyz_filtered_cloud));
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

            /* ========================================
             * Fill Code: EUCLIDEAN CLUSTER EXTRACTION
             * ========================================*/
            // Creating the KdTree object for the search method of the extraction
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
            *cloud_filtered = *cloud_f;
            tree->setInputCloud (cloud_filtered);

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance (cluster_tol);
            ec.setMinClusterSize (cluster_min_size);
            ec.setMaxClusterSize (cluster_max_size);
            ec.setSearchMethod (tree);
            ec.setInputCloud (cloud_filtered);
            ec.extract (cluster_indices);

            std::vector<sensor_msgs::msg::PointCloud2::SharedPtr> pc2_clusters;
            std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > clusters;

            int j = 0;
            for (const auto& cluster : cluster_indices)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

                for (const auto& idx : cluster.indices) {
                    cloud_cluster->points.push_back((*cloud_filtered)[idx]);
                }

                cloud_cluster->width = cloud_cluster->points.size ();
                cloud_cluster->height = 1;
                cloud_cluster->is_dense = true;
                RCLCPP_INFO(this->get_logger(), "Cluster has '%lu' points", cloud_cluster->points.size());
                clusters.push_back(cloud_cluster);
                sensor_msgs::msg::PointCloud2::SharedPtr tempROSMsg(new sensor_msgs::msg::PointCloud2);
                pcl::toROSMsg(*cloud_cluster, *tempROSMsg);
                pc2_clusters.push_back(tempROSMsg);

                j++;

            }


            /* ========================================
             * Fill Code: STATISTICAL OUTLIER REMOVAL
             * ========================================*/
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud_ptr= clusters.at(0);
            pcl::PointCloud<pcl::PointXYZ>::Ptr sor_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud (cluster_cloud_ptr);
            sor.setMeanK (50);
            sor.setStddevMulThresh (1.0);
            sor.filter (*sor_cloud_filtered);


            /* ========================================
             * BROADCAST TRANSFORM
             * ========================================*/
            std::unique_ptr<tf2_ros::TransformBroadcaster> br = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            geometry_msgs::msg::TransformStamped part_transform;

            tf2::Quaternion q;
            q.setRPY(0, 0, 0);
            part_transform.transform.rotation.x = q.x();
            part_transform.transform.rotation.y = q.y();
            part_transform.transform.rotation.z = q.z();
            part_transform.transform.rotation.w = q.w();

            //Here x,y, and z should be calculated based on the PointCloud2 filtering results
            part_transform.transform.translation.x = sor_cloud_filtered->at(1).x;
            part_transform.transform.translation.y = sor_cloud_filtered->at(1).y;
            part_transform.transform.translation.z = sor_cloud_filtered->at(1).z;
            part_transform.header.stamp = this->get_clock()->now();
            part_transform.header.frame_id = world_frame;
            part_transform.child_frame_id = "part";

            br->sendTransform(part_transform);

            /* ========================================
             * Fill Code: POLYGONAL SEGMENTATION
             * ========================================*/
            pcl::PointCloud<pcl::PointXYZ>::Ptr sensor_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>(cloud));
            pcl::PointCloud<pcl::PointXYZ>::Ptr prism_filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr pick_surface_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
            pcl::ExtractIndices<pcl::PointXYZ> extract_ind;
            prism.setInputCloud(sensor_cloud_ptr);
            pcl::PointIndices::Ptr pt_inliers (new pcl::PointIndices());

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

            Eigen::Affine3d eigen3d = tf2::transformToEigen(part_transform);
            pcl::transformPointCloud(*pick_surface_cloud_ptr,*pick_surface_cloud_ptr,Eigen::Affine3f(eigen3d));

            prism.setInputPlanarHull( pick_surface_cloud_ptr);
            prism.setHeightLimits(-10,10);

            prism.segment(*pt_inliers);

            extract_ind.setInputCloud(sensor_cloud_ptr);
            extract_ind.setIndices(pt_inliers);

            extract_ind.setNegative(true);
            extract_ind.filter(*prism_filtered_cloud);

            /* ========================================
             * CONVERT PointCloud2 PCL->ROS
             * PUBLISH CLOUD
             * Fill Code: UPDATE AS NECESSARY
             * ========================================*/

            this->publishPointCloud(voxel_grid_publisher_, *cloud_voxel_filtered);
            this->publishPointCloud(passthrough_publisher_, zf_cloud); // this->publishPointCloud(passthrough_publisher_, xyz_filtered_cloud);
            this->publishPointCloud(plane_publisher_, *cloud_f);
            this->publishPointCloud(euclidean_publisher_, *(clusters.at(0)));
            this->publishPointCloud(stat_publisher_, *sor_cloud_filtered);
            this->publishPointCloud(polygon_publisher_, *prism_filtered_cloud);

        }

        void publishPointCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher,
                               pcl::PointCloud<pcl::PointXYZ> point_cloud)
        {

            sensor_msgs::msg::PointCloud2::SharedPtr pc2_cloud(new sensor_msgs::msg::PointCloud2);
            pcl::toROSMsg(point_cloud, *pc2_cloud);
            pc2_cloud->header.frame_id = world_frame;
            pc2_cloud->header.stamp = this->get_clock()->now();
            publisher->publish(*pc2_cloud);
        }

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;

        /*
         * Publishers
         */
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_grid_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr passthrough_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr plane_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr euclidean_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr stat_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr polygon_publisher_;

        /*
         * Parameters
         */
        std::string cloud_topic;
        std::string world_frame;
        std::string camera_frame;
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
         * TF
         */
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> br;
};


int main(int argc, char *argv[])
{
    /*
     * INITIALIZE ROS NODE
     */
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PerceptionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
