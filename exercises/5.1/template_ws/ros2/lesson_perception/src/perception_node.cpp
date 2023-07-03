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
#include <tf2_eigen/tf2_eigen.hpp>
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


            /*
             * SET UP PARAMETERS
             */
            rclcpp::Parameter cloud_topic_param, world_frame_param, camera_frame_param;

            RCLCPP_INFO(this->get_logger(), "Getting parameters");

            this->get_parameter_or("cloud_topic", cloud_topic_param, rclcpp::Parameter("", "/kinect/depth_registered/points"));
            this->get_parameter_or("world_frame", world_frame_param, rclcpp::Parameter("", "kinect_link"));
            this->get_parameter_or("camera_frame", camera_frame_param, rclcpp::Parameter("", "kinect_link"));


            cloud_topic = cloud_topic_param.as_string();
            world_frame = world_frame_param.as_string();
            camera_frame = camera_frame_param.as_string();


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


            /* ========================================
             * Fill Code: PASSTHROUGH FILTER(S)
             * ========================================*/


            /* ========================================
             * Fill Code: CROPBOX (Optional)
             * ========================================*/


            /* ========================================
             * Fill Code: PLANE SEGEMENTATION
             * ========================================*/



            /* ========================================
             * Fill Code: EUCLIDEAN CLUSTER EXTRACTION
             * ========================================*/



            /* ========================================
             * Fill Code: STATISTICAL OUTLIER REMOVAL
             * ========================================*/


            /* ========================================
             * Fill Code: PUBLISH OTHER MARKERS (OPTIONAL)
             * ========================================*/


            /* ========================================
             * BROADCAST TRANSFORM
             * ========================================*/


            /* ========================================
             * Fill Code: POLYGONAL SEGMENTATION
             * ========================================*/


            /* ========================================
             * CONVERT PointCloud2 PCL->ROS
             * PUBLISH CLOUD
             * Fill Code: UPDATE AS NECESSARY
             * ========================================*/

            this->publishPointCloud(voxel_grid_publisher_, cloud);

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


        /*
         * Parameters
         */
        std::string cloud_topic;
        std::string world_frame;
        std::string camera_frame;

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
