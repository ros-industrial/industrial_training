/*
 * target_recognition.cpp
 *
 *  Created on: Jan 16, 2014
 *      Author: ros-industrial
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
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
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <collision_avoidance_pick_and_place/GetTargetPose.h>
#include <math.h>

// alias
typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
typedef boost::shared_ptr<tf::TransformListener> TransformListenerPtr;

// constants
const std::string FILTERED_CLOUD_TOPIC = "filtered_cloud";
const std::string TARGET_RECOGNITION_SERVICE = "target_recognition";
const std::string BOX_PICK_FRAME_ID = "box_pick_frame";
const float BOX_HEIGHT_TOLERANCE = 0.02f;
const float BOX_PADDING_SCALE = 1.8f;

// global variables
std::string AR_TAG_FRAME_ID = "";
std::string WORLD_FRAME_ID = "";
float BOX_WIDTH;
float BOX_LENGTH;
float BOX_HEIGHT;
sensor_msgs::PointCloud2 FILTERED_CLOUD_MSG;
sensor_msgs::PointCloud2 SENSOR_CLOUD_MSG;

// ros parameter
std::string SENSOR_CLOUD_TOPIC = "sensor_cloud";

// ros service server
ros::ServiceServer target_detection_server;

// ros subscriber
ros::Subscriber point_cloud_subscriber;

// ros publishers and subscribers
ros::Publisher filtered_cloud_publisher;

// transform listener
TransformListenerPtr transform_listener_ptr;

// function signatures
bool target_recognition_callback(collision_avoidance_pick_and_place::GetTargetPose::Request& req,
		collision_avoidance_pick_and_place::GetTargetPose::Response& res);
void point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr msg);
bool grab_sensor_snapshot(sensor_msgs::PointCloud2& msg);
bool get_transform(std::string target,std::string source,tf::Transform& trg_to_src_tf);
void filter_box(const tf::Transform& world_to_sensor_tf,
		const tf::Transform& world_to_box_pick_tf,const Cloud &sensor_cloud,
		Cloud& filtered_cloud);
bool detect_box_height(const Cloud& cloud,
		const tf::Transform &world_to_ar_tf,double& height );

// main program
int main(int argc,char** argv)
{
	ros::init(argc,argv,"target_recognition_node");
	ros::NodeHandle nh;
	ros::NodeHandle ph("~");

	// read parameters
	if(ph.getParam("sensor_topic",SENSOR_CLOUD_TOPIC))
	{
		ROS_INFO_STREAM("target recognition read parameters successfully");
	}
	else
	{
		ROS_ERROR_STREAM("target recognition did not find parameters");
	}

	// initializing service server
	target_detection_server = nh.advertiseService(TARGET_RECOGNITION_SERVICE,target_recognition_callback);

	// initializing publisher
	filtered_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>(FILTERED_CLOUD_TOPIC,1);

	// initializing subscriber
	point_cloud_subscriber = nh.subscribe(SENSOR_CLOUD_TOPIC,1,point_cloud_callback);

	// initializing transform listener
	transform_listener_ptr = TransformListenerPtr(new tf::TransformListener(nh,ros::Duration(1.0f)));

	// initializing cloud messages
	FILTERED_CLOUD_MSG = sensor_msgs::PointCloud2();

	while(ros::ok())
	{
		ros::Duration(0.2f).sleep();
		ros::spinOnce();
		//filtered_cloud_publisher.publish(FILTERED_CLOUD_MSG);
	}

	return 0;
}

bool grab_sensor_snapshot(sensor_msgs::PointCloud2& msg)
{
	// grab sensor data snapshot
	ros::NodeHandle nh;
	sensor_msgs::PointCloud2ConstPtr msg_ptr =
			ros::topic::waitForMessage<sensor_msgs::PointCloud2>(SENSOR_CLOUD_TOPIC,nh,
					ros::Duration(5.0f));

	// check for empty message
	if(msg_ptr != sensor_msgs::PointCloud2ConstPtr())
	{

		msg = *msg_ptr;
	}

	return msg_ptr != sensor_msgs::PointCloud2ConstPtr();
}

void point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr msg)
{
	SENSOR_CLOUD_MSG = sensor_msgs::PointCloud2(*msg);
	//SENSOR_CLOUD_MSG = *msg;
}

bool target_recognition_callback(collision_avoidance_pick_and_place::GetTargetPose::Request& req,
		collision_avoidance_pick_and_place::GetTargetPose::Response& res)
{
	// transform listener and broadcaster
	//tf::TransformListener tf_listener;

	// transforms
	tf::StampedTransform world_to_sensor_tf;
	tf::Transform world_to_box_pick_tf;
	tf::Transform world_to_ar_tf;
	tf::Vector3 box_pick_position;

	// updating global variables
	BOX_LENGTH = req.shape.dimensions[0];
	BOX_WIDTH = req.shape.dimensions[1];
	BOX_HEIGHT = req.shape.dimensions[2];
	WORLD_FRAME_ID = req.world_frame_id;
	AR_TAG_FRAME_ID = req.ar_tag_frame_id;

	// get point cloud message
	sensor_msgs::PointCloud2 msg = SENSOR_CLOUD_MSG;
	if(msg.data.size() == 0)
	{
			ROS_ERROR_STREAM("Cloud message is invalid, returning detection failure");
			res.succeeded = false;
			return true;
	}

	// looking up transforms
	if(get_transform(WORLD_FRAME_ID , AR_TAG_FRAME_ID,world_to_ar_tf) &&
			get_transform(WORLD_FRAME_ID,msg.header.frame_id,world_to_sensor_tf))
	{


		// convert from message to point cloud
		Cloud::Ptr sensor_cloud_ptr(new Cloud());
		pcl::fromROSMsg<pcl::PointXYZ>(msg,*sensor_cloud_ptr);

		// applying statistical removal
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		sor.setInputCloud (sensor_cloud_ptr);
		sor.setMeanK (50);
		sor.setStddevMulThresh (1.0);
		sor.filter (*sensor_cloud_ptr);

		// creating cloud message for publisher
		Cloud::Ptr filtered_cloud_ptr(new Cloud());

		// converting to world coordinates
		Eigen::Affine3d eigen_3d;
		tf::transformTFToEigen(world_to_sensor_tf,eigen_3d);
		Eigen::Affine3f eigen_3f(eigen_3d);
		pcl::transformPointCloud(*sensor_cloud_ptr,*sensor_cloud_ptr,eigen_3f);

		// find height of box top surface
		double height;
		if(!detect_box_height(*sensor_cloud_ptr,world_to_ar_tf,height) )
		{
			ROS_ERROR_STREAM("Target height detection failed");
			res.succeeded = false;
			return true;
		}

		// updating box pick transform
		world_to_box_pick_tf = world_to_ar_tf;
		box_pick_position = world_to_box_pick_tf.getOrigin();
		box_pick_position.setZ(height);
		world_to_box_pick_tf.setOrigin(box_pick_position);

		// filtering box from sensor cloud
		filter_box(world_to_sensor_tf,
				world_to_box_pick_tf,*sensor_cloud_ptr,*filtered_cloud_ptr);

		// filter box at requested locations
		for(unsigned int i =0;i < req.remove_at_poses.size();i++)
		{
			tf::Transform world_to_box;
			tf::poseMsgToTF(req.remove_at_poses[i],world_to_box);

			// copying last filter cloud
			pcl::copyPointCloud(*filtered_cloud_ptr,*sensor_cloud_ptr);

			// filter box from last computed cloud
			filter_box(world_to_sensor_tf,world_to_box,*sensor_cloud_ptr,*filtered_cloud_ptr);
		}

		// transforming to world frame
		pcl::transformPointCloud(*filtered_cloud_ptr,*filtered_cloud_ptr,eigen_3f.inverse());

		// converting to message
		FILTERED_CLOUD_MSG = sensor_msgs::PointCloud2();
		pcl::toROSMsg(*filtered_cloud_ptr,FILTERED_CLOUD_MSG);

		// populating response
		tf::poseTFToMsg(world_to_box_pick_tf,res.target_pose);
		res.succeeded = true ;

		// publishing cloud
		FILTERED_CLOUD_MSG.header.stamp = ros::Time::now()-ros::Duration(0.5f);
		filtered_cloud_publisher.publish(FILTERED_CLOUD_MSG);
	}
	else
	{
		res.succeeded = false;
	}

	return true;
}

bool get_transform(std::string target,std::string source,tf::Transform& trg_to_src_tf)
{
	// create tf listener and broadcaster
	//static tf::TransformListener tf_listener;
	tf::StampedTransform trg_to_src_stamped;

	// find ar tag transform
	try
	{
		transform_listener_ptr->waitForTransform(target,source,ros::Time::now(),ros::Duration(4.0f));
		transform_listener_ptr->lookupTransform(target,source,ros::Time::now() - ros::Duration(0.2f),trg_to_src_stamped);

		// copying transform data
		trg_to_src_tf.setRotation( trg_to_src_stamped.getRotation());
		trg_to_src_tf.setOrigin(trg_to_src_stamped.getOrigin());

	}
	catch(tf::LookupException &e)
	{
		ROS_ERROR_STREAM("transform lookup for '"<<AR_TAG_FRAME_ID<<"' failed");
		return false;
	}
	catch(tf::ExtrapolationException &e)
	{
		ROS_ERROR_STREAM("transform lookup for '"<<AR_TAG_FRAME_ID<<"' failed");
		return false;
	}
	catch(tf::TransformException &e)
	{
		ROS_ERROR_STREAM("transform lookup for '"<<AR_TAG_FRAME_ID<<"' failed");
		return false;
	}

	return true;
}

bool detect_box_height(const Cloud& cloud,
		const tf::Transform &world_to_ar_tf,double& height )
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
	int count = pcl::compute3DCentroid(*filtered_cloud_ptr,centroid);
	height = centroid[2];

	ROS_INFO_STREAM("Detected height is: "<<centroid[2]);

	// return z value
	return count != 0;
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
	//prism.setViewPoint(viewpoint.x(),viewpoint.y(),viewpoint.z());
	prism.segment(*inliers);

	//pcl::copyPointCloud(*sensor_cloud_ptr,indices.indices,filtered_cloud);
	// extracting remaining points
	extract.setInputCloud(sensor_cloud_ptr);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(filtered_cloud);

	ROS_INFO_STREAM("Filtered cloud points: "<<filtered_cloud.points.size());

}

