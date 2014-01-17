/*
 * target_recognition.cpp
 *
 *  Created on: Jan 16, 2014
 *      Author: ros-industrial
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/centroid.h>
#include <pcl_ros/transforms.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/common.h>
#include <boost/make_shared.hpp>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <math.h>

// alias
typedef pcl::PointCloud<pcl::PointXYZ> Cloud;

// constants
const std::string SENSOR_CLOUD_TOPIC = "sensor_cloud";
const std::string FILTERED_CLOUD_TOPIC = "filtered_cloud";
const std::string BOX_PICK_FRAME_ID = "box_pick_frame";
const float BOX_PADDING_SCALE = 1.5f;

// parameters
std::string AR_TAG_FRAME_ID = "";
std::string WORLD_FRAME_ID = "";
double BOX_WIDTH;
double BOX_LENGTH;

// ros publishers and subscribers
ros::Publisher filtered_cloud_pubisher;
ros::Subscriber sensor_cloud_subscriber;

void point_cloud_callback(sensor_msgs::PointCloud2ConstPtr msg);
bool get_ar_transform(tf::Transform& world_to_ar_tf);
void filter_box(const tf::Transform& world_to_sensor_tf,
		const tf::Transform& world_to_box_pick_tf,const Cloud &sensor_cloud,
		Cloud& filtered_cloud);
double detect_box_height(const Cloud& cloud,
		const tf::Transform &world_to_ar_tf);

int main(int argc,char** argv)
{
	ros::init(argc,argv,"target_recognition_node");
	ros::NodeHandle nh("~");
	ros::NodeHandle ph; // public handle

	// load parameters
	if(nh.getParam("tag_frame_id",AR_TAG_FRAME_ID) &&
			nh.getParam("world_frame_id",WORLD_FRAME_ID) &&
			nh.getParam("box_width",BOX_WIDTH) &&
			nh.getParam("box_length",BOX_LENGTH))
	{
		ROS_INFO_STREAM("Parameters loaded");
	}
	else
	{
		ROS_ERROR_STREAM("Parameters not found, exiting");
		return 0;
	}

	// initializing publisher and subscriber
	sensor_cloud_subscriber = ph.subscribe(SENSOR_CLOUD_TOPIC,1,point_cloud_callback);
	filtered_cloud_pubisher = ph.advertise<sensor_msgs::PointCloud2>(FILTERED_CLOUD_TOPIC,1);
	while(ros::ok())
	{
		//ros::Duration(0.2f).sleep();
		ros::spin();
	}

	return 0;
}

bool get_ar_transform(tf::Transform& world_to_ar_tf)
{
	// create tf listener and broadcaster
	static tf::TransformListener tf_listener;
	tf::StampedTransform world_to_ar_tf_stamped;

	// find ar tag transform
	try
	{
		tf_listener.lookupTransform(WORLD_FRAME_ID,AR_TAG_FRAME_ID,
				ros::Time(0),world_to_ar_tf_stamped);

		// copying transform data
		world_to_ar_tf.setRotation( world_to_ar_tf_stamped.getRotation());
		world_to_ar_tf.setOrigin(world_to_ar_tf_stamped.getOrigin());

	}
	catch(tf::LookupException &e)
	{
		return false;
	}
	catch(tf::ExtrapolationException &e)
	{
		return false;
	}

	return true;
}


double detect_box_height(const Cloud& cloud,
		const tf::Transform &world_to_ar_tf)
{
	// cloud objects
	Cloud::Ptr cloud_ptr= boost::make_shared<Cloud>(cloud);
	Cloud::Ptr filtered_cloud_ptr(new Cloud);

	// applying filter in x axis
	float min = world_to_ar_tf.getOrigin().x() - 0.2f*BOX_LENGTH;
	float max = world_to_ar_tf.getOrigin().x() + 0.2f*BOX_LENGTH;
	pcl::PassThrough<pcl::PointXYZ> filter;
	filter.setInputCloud(cloud_ptr);
	filter.setFilterFieldName("x");
	filter.setFilterLimits(min,max);
	filter.filter(*filtered_cloud_ptr);

	// applying filter in y axis
	min = world_to_ar_tf.getOrigin().y()- 0.2f*BOX_WIDTH;
	max = world_to_ar_tf.getOrigin().y() + 0.2f*BOX_WIDTH;;
	filter.setInputCloud(filtered_cloud_ptr);
	filter.setFilterFieldName("y");
	filter.setFilterLimits(min,max);
	filter.filter(*filtered_cloud_ptr);

	// computing centroid
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*filtered_cloud_ptr,centroid);

	ROS_WARN_STREAM("Detected height is: "<<centroid[2]);

	// return z value
	return centroid[2];

}

void filter_box(const tf::Transform& world_to_sensor_tf,
		const tf::Transform& world_to_box_pick_tf,const Cloud &sensor_cloud,
		Cloud& filtered_cloud)
{
	// creating surface with center at box pick location
	float x = world_to_box_pick_tf.getOrigin().x();
	float y = world_to_box_pick_tf.getOrigin().y();
	float z = world_to_box_pick_tf.getOrigin().z();
	Cloud::Ptr pick_surface_cloud_ptr(new Cloud());

	// adding surface points to cloud
	pick_surface_cloud_ptr->width = 4;
	pick_surface_cloud_ptr->height = 1;
	pick_surface_cloud_ptr->points.resize(4);

	pick_surface_cloud_ptr->points[0].x = x + 0.5f*BOX_PADDING_SCALE*BOX_LENGTH;
	pick_surface_cloud_ptr->points[0].y = y + 0.5f*BOX_PADDING_SCALE*BOX_WIDTH;
	pick_surface_cloud_ptr->points[0].z = z;

	pick_surface_cloud_ptr->points[1].x = x - 0.5f*BOX_PADDING_SCALE*BOX_LENGTH;
	pick_surface_cloud_ptr->points[1].y = y + 0.5f*BOX_PADDING_SCALE*BOX_WIDTH;
	pick_surface_cloud_ptr->points[1].z = z;

	pick_surface_cloud_ptr->points[2].x = x - 0.5f*BOX_PADDING_SCALE*BOX_LENGTH;
	pick_surface_cloud_ptr->points[2].y = y - 0.5f*BOX_PADDING_SCALE*BOX_WIDTH;
	pick_surface_cloud_ptr->points[2].z = z;

	pick_surface_cloud_ptr->points[3].x = x + 0.5f*BOX_PADDING_SCALE*BOX_LENGTH;
	pick_surface_cloud_ptr->points[3].y = y - 0.5f*BOX_PADDING_SCALE*BOX_WIDTH;
	pick_surface_cloud_ptr->points[3].z = z;

	// create hull
	ROS_INFO_STREAM("Points in surface: "<<pick_surface_cloud_ptr->points.size());

	// extracting points in extruded volume
	Cloud::Ptr sensor_cloud_ptr = boost::make_shared<Cloud>(sensor_cloud);
	pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

	ROS_INFO_STREAM("Sensor cloud points: "<<sensor_cloud_ptr->points.size());

	tf::Vector3 viewpoint = world_to_sensor_tf.getOrigin();
	prism.setInputCloud(sensor_cloud_ptr);
	prism.setInputPlanarHull( pick_surface_cloud_ptr);
	prism.setHeightLimits(-10,10);
	prism.setViewPoint(viewpoint.x(),viewpoint.y(),viewpoint.z());
	prism.segment(*inliers);

	//pcl::copyPointCloud(*sensor_cloud_ptr,indices.indices,filtered_cloud);
	// extracting remaining points
	extract.setInputCloud(sensor_cloud_ptr);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(filtered_cloud);

	ROS_INFO_STREAM("Filtered cloud points: "<<filtered_cloud.points.size());

}

void point_cloud_callback(sensor_msgs::PointCloud2ConstPtr msg)
{
	static tf::TransformListener tf_listener;
	static tf::TransformBroadcaster tf_broadcaster;

	// convert from message to point cloud
	Cloud::Ptr sensor_cloud_ptr(new Cloud());
    pcl::fromROSMsg<pcl::PointXYZ>(*msg,*sensor_cloud_ptr);

    // applying statistical removal
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (sensor_cloud_ptr);
	sor.setMeanK (50);
	sor.setStddevMulThresh (1.0);
	sor.filter (*sensor_cloud_ptr);

    // creating cloud message for publisher
    Cloud::Ptr filtered_cloud_ptr(new Cloud());
    sensor_msgs::PointCloud2 filtered_cloud_msg;

    // detecting ar tag
    static tf::StampedTransform world_to_box_pick_tf;
    static tf::StampedTransform world_to_sensor_tf;
    tf::Transform world_to_ar_tf = tf::Transform::getIdentity();
    tf::Vector3 pos;
    if(get_ar_transform(world_to_ar_tf))
    {

		// getting sensor transform
		try
		{
			tf_listener.lookupTransform(WORLD_FRAME_ID,msg->header.frame_id,
							ros::Time(0),world_to_sensor_tf);
		}
		catch(tf::LookupException &e)
		{
			//return;
		}
		catch(tf::ExtrapolationException &e)
		{
			//return;
		}

		// converting to world coordinates
		pcl_ros::transformPointCloud<pcl::PointXYZ>(WORLD_FRAME_ID,
			  *sensor_cloud_ptr,*sensor_cloud_ptr,tf_listener);

		float height = detect_box_height(*sensor_cloud_ptr,world_to_ar_tf);

		// broadcasting box pick transform
		world_to_box_pick_tf = tf::StampedTransform(world_to_ar_tf,
			  ros::Time(0),WORLD_FRAME_ID,BOX_PICK_FRAME_ID);
		pos = world_to_box_pick_tf.getOrigin();
		pos.setZ(height);
		world_to_box_pick_tf.setOrigin(pos);
		tf_broadcaster.sendTransform(world_to_box_pick_tf);
	}

	// filtering box from sensor cloud
	filter_box(world_to_sensor_tf,world_to_box_pick_tf,*sensor_cloud_ptr,*filtered_cloud_ptr);

	// converting to message
	pcl::toROSMsg(*filtered_cloud_ptr,filtered_cloud_msg);

	// publishing
	filtered_cloud_pubisher.publish(filtered_cloud_msg);

}