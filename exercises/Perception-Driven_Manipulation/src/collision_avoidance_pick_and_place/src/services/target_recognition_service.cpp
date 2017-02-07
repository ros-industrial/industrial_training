
#ifdef __i386__
  #pragma message("i386 Architecture detected, disabling EIGEN VECTORIZATION")
  #define EIGEN_DONT_VECTORIZE
  #define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#else
  #pragma message("64bit Architecture detected, enabling EIGEN VECTORIZATION")
#endif



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
const std::string SENSOR_CLOUD_TOPIC = "sensor_cloud";
const std::string FILTERED_CLOUD_TOPIC = "filtered_cloud";
const std::string TARGET_RECOGNITION_SERVICE = "target_recognition";

// defaults
const double BOX_SCALE = 1.2f;
const double ANGLE_TOLERANCE = 0.1f* (M_PI/180.0f);


class TargetRecognition
{
public:
	TargetRecognition():
		box_filter_scale_(BOX_SCALE),
		angle_tolerance_(ANGLE_TOLERANCE)
	{

	}

	~TargetRecognition()
	{

	}

	bool init()
	{
		ros::NodeHandle nh;
		ros::NodeHandle ph("~");

		// read parameters
		if(ph.getParam("box_filter_scale",box_filter_scale_) &&
				ph.getParam("angle_tolerance",angle_tolerance_))
		{
			ROS_INFO_STREAM("target recognition read parameters successfully");
		}
		else
		{
			ROS_WARN_STREAM("target recognition did not find one or more parameters, using defaults");
		}

		// initializing service server
		target_detection_server = nh.advertiseService(TARGET_RECOGNITION_SERVICE,&TargetRecognition::target_recognition_callback,this);

		// initializing publisher
		filtered_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>(FILTERED_CLOUD_TOPIC,1);

		// initializing subscriber
		point_cloud_subscriber = nh.subscribe(SENSOR_CLOUD_TOPIC,1,&TargetRecognition::point_cloud_callback,this);

		// initializing transform listener
		transform_listener_ptr = TransformListenerPtr(new tf::TransformListener(nh,ros::Duration(1.0f)));

		// initializing cloud messages
		filtered_cloud_msg_ = sensor_msgs::PointCloud2();

		return true;

	}

	void run()
	{

		while(ros::ok())
		{
			ros::Duration(0.2f).sleep();
			ros::spinOnce();
		}
	}

protected:

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
		sensor_cloud_msg_ = sensor_msgs::PointCloud2(*msg);
	}

	bool target_recognition_callback(collision_avoidance_pick_and_place::GetTargetPose::Request& req,
			collision_avoidance_pick_and_place::GetTargetPose::Response& res)
	{
		// transforms
		tf::StampedTransform world_to_sensor_tf;
		tf::Transform world_to_box_pick_tf;
		tf::Transform world_to_ar_tf;
		tf::Vector3 box_pick_position;

		// updating global variables
		box_length_ = req.shape.dimensions[0];
		box_width_ = req.shape.dimensions[1];
		box_height_ = req.shape.dimensions[2];
		world_frame_id_ = req.world_frame_id;
		ar_frame_id_ = req.ar_tag_frame_id;

		// get point cloud message
		sensor_msgs::PointCloud2 msg = sensor_cloud_msg_;
		if(msg.data.size() == 0)
		{
				ROS_ERROR_STREAM("Cloud message is invalid, returning detection failure");
				res.succeeded = false;
				return true;
		}

		// looking up transforms
		if(get_transform(world_frame_id_ , ar_frame_id_,world_to_ar_tf) &&
				get_transform(world_frame_id_,msg.header.frame_id,world_to_sensor_tf))
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
			filtered_cloud_msg_ = sensor_msgs::PointCloud2();
			pcl::toROSMsg(*filtered_cloud_ptr,filtered_cloud_msg_);

			// populating response
			tf::poseTFToMsg(world_to_box_pick_tf,res.target_pose);
			res.succeeded = true ;

			// publishing cloud
			filtered_cloud_msg_.header.stamp = ros::Time::now()-ros::Duration(0.5f);
			filtered_cloud_publisher.publish(filtered_cloud_msg_);
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
			ROS_ERROR_STREAM("transform lookup for '"<<ar_frame_id_<<"' failed");
			return false;
		}
		catch(tf::ExtrapolationException &e)
		{
			ROS_ERROR_STREAM("transform lookup for '"<<ar_frame_id_<<"' failed");
			return false;
		}
		catch(tf::TransformException &e)
		{
			ROS_ERROR_STREAM("transform lookup for '"<<ar_frame_id_<<"' failed");
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
		float min = world_to_ar_tf.getOrigin().x() - 0.2f*box_length_;
		float max = world_to_ar_tf.getOrigin().x() + 0.2f*box_length_;
		pcl::PassThrough<pcl::PointXYZ> filter;
		filter.setInputCloud(cloud_ptr);
		filter.setFilterFieldName("x");
		filter.setFilterLimits(min,max);
		filter.filter(*filtered_cloud_ptr);

		// applying filter in y axis
		min = world_to_ar_tf.getOrigin().y()- 0.2f*box_width_;
		max = world_to_ar_tf.getOrigin().y() + 0.2f*box_width_;;
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
	Cloud::Ptr pick_surface_cloud_ptr(new Cloud());

	// adding surface points to cloud
	pick_surface_cloud_ptr->width = 5;
	pick_surface_cloud_ptr->height = 1;
	pick_surface_cloud_ptr->points.resize(5);

	pick_surface_cloud_ptr->points[0].x = 0.5f*box_filter_scale_*box_length_;
	pick_surface_cloud_ptr->points[0].y = 0.5f*box_filter_scale_*box_width_;
	pick_surface_cloud_ptr->points[0].z = 0;

	pick_surface_cloud_ptr->points[1].x = -0.5f*box_filter_scale_*box_length_;
	pick_surface_cloud_ptr->points[1].y = 0.5f*box_filter_scale_*box_width_;
	pick_surface_cloud_ptr->points[1].z = 0;

	pick_surface_cloud_ptr->points[2].x = -0.5f*box_filter_scale_*box_length_;
	pick_surface_cloud_ptr->points[2].y = -0.5f*box_filter_scale_*box_width_;
	pick_surface_cloud_ptr->points[2].z = 0;

	pick_surface_cloud_ptr->points[3].x = 0.5f*box_filter_scale_*box_length_;
	pick_surface_cloud_ptr->points[3].y = -0.5f*box_filter_scale_*box_width_;
	pick_surface_cloud_ptr->points[3].z = 0;

	pick_surface_cloud_ptr->points[4].x = 0.5f*box_filter_scale_*box_length_;
	pick_surface_cloud_ptr->points[4].y = 0.5f*box_filter_scale_*box_width_;
	pick_surface_cloud_ptr->points[4].z = 0;

	ROS_INFO_STREAM("Points in surface: "<<pick_surface_cloud_ptr->points.size());

	// finding angle between world z and box z vectors
	tf::Vector3 z_world_vect(0,0,1);
	tf::Vector3 z_box_vect = world_to_box_pick_tf.getBasis().getColumn(2);
	double angle  = z_world_vect.angle(z_box_vect);
	ROS_INFO_STREAM("Angle between z vectors : "<<angle);


	// transforming cloud to match orientation of box (rectify in the z direction)
	tf::Vector3 axis = z_world_vect.cross(z_box_vect);
	tf::Transform world_to_rectified_box_tf = world_to_box_pick_tf;
	Eigen::Affine3d eigen3d;
	if(std::abs(angle) > angle_tolerance_)
	{
		ROS_INFO_STREAM("Rectifying pick pose z direction");
		world_to_rectified_box_tf.setRotation(world_to_box_pick_tf.getRotation() * tf::Quaternion(axis,-angle)) ;
	}

	tf::transformTFToEigen(world_to_rectified_box_tf,eigen3d);
	pcl::transformPointCloud(*pick_surface_cloud_ptr,*pick_surface_cloud_ptr,Eigen::Affine3f(eigen3d));


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

protected:

	// members
	std::string ar_frame_id_;
	std::string world_frame_id_;
	float box_width_;
	float box_length_;
	float box_height_;
	sensor_msgs::PointCloud2 filtered_cloud_msg_;
	sensor_msgs::PointCloud2 sensor_cloud_msg_;

	// ros parameter
	double box_filter_scale_;
	double angle_tolerance_;

	// ros service server
	ros::ServiceServer target_detection_server;

	// ros subscriber
	ros::Subscriber point_cloud_subscriber;

	// ros publishers and subscribers
	ros::Publisher filtered_cloud_publisher;

	// transform listener
	TransformListenerPtr transform_listener_ptr;

};
// main program
int main(int argc,char** argv)
{
	ros::init(argc,argv,"target_recognition_node");

	TargetRecognition tg;
	if(tg.init())
	{
		tg.run();
	}

	return 0;
}
