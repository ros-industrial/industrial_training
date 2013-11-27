/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PointStamped.h>

#include <turtlebot_kinect_arm_calibration/detect_calibration_pattern.h>

using namespace std;
using namespace Eigen;

tf::Transform tfFromEigen(Eigen::Matrix4f trans)
{
    tf::Matrix3x3 btm;
    btm.setValue(trans(0,0),trans(0,1),trans(0,2),
                 trans(1,0),trans(1,1),trans(1,2),
                 trans(2,0),trans(2,1),trans(2,2));
    tf::Transform ret;
    ret.setOrigin(tf::Vector3(trans(0,3),trans(1,3),trans(2,3)));
    ret.setBasis(btm);
    return ret;
}

Eigen::Matrix4f EigenFromTF(tf::Transform trans)
{
  Eigen::Matrix4f out;
  tf::Quaternion quat = trans.getRotation();
  tf::Vector3 origin = trans.getOrigin();
  
  Eigen::Quaternionf quat_out(quat.w(), quat.x(), quat.y(), quat.z());
  Eigen::Vector3f origin_out(origin.x(), origin.y(), origin.z());
  
  out.topLeftCorner<3,3>() = quat_out.toRotationMatrix();
  out.topRightCorner<3,1>() = origin_out;
  out(3,3) = 1;
  
  return out;
}


class CalibrateKinectCheckerboard
{
    // Nodes and publishers/subscribers
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher pub_;
    ros::Publisher detector_pub_;
    ros::Publisher physical_pub_;
  
    // Image and camera info subscribers;
    ros::Subscriber image_sub_; 
    ros::Subscriber info_sub_;

    // Structures for interacting with ROS messages
    cv_bridge::CvImagePtr input_bridge_;
    cv_bridge::CvImagePtr output_bridge_;
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;
    image_geometry::PinholeCameraModel cam_model_;
    
    // Calibration objects
    PatternDetector pattern_detector_;
    
    // The optimized transform
    Eigen::Transform<float, 3, Eigen::Affine> transform_;
    
    // Visualization for markers
    pcl::PointCloud<pcl::PointXYZ> detector_points_;
    pcl::PointCloud<pcl::PointXYZ> ideal_points_;
    pcl::PointCloud<pcl::PointXYZ> image_points_;
    pcl::PointCloud<pcl::PointXYZ> physical_points_;
    
    // Have we calibrated the camera yet?
    bool calibrated;
    
    ros::Timer timer_;
    
    // Parameters
    std::string fixed_frame;
    std::string camera_frame;
    std::string target_frame;
    std::string tip_frame;
    std::string touch_frame;
    
    int checkerboard_width;
    int checkerboard_height;
    double checkerboard_grid;
    
    // Gripper tip position
    geometry_msgs::PointStamped gripper_tip;

public:
  CalibrateKinectCheckerboard()
    : nh_("~"), it_(nh_), calibrated(false)
  {
    // Load parameters from the server.
    nh_.param<std::string>("fixed_frame", fixed_frame, "/base_link");
    nh_.param<std::string>("camera_frame", camera_frame, "/camera_link");
    nh_.param<std::string>("target_frame", target_frame, "/calibration_pattern");
    nh_.param<std::string>("tip_frame", tip_frame, "/gripper_link");
    
    nh_.param<int>("checkerboard_width", checkerboard_width, 6);
    nh_.param<int>("checkerboard_height", checkerboard_height, 7);
    nh_.param<double>("checkerboard_grid", checkerboard_grid, 0.027);
    
    // Set pattern detector sizes
    pattern_detector_.setPattern(cv::Size(checkerboard_width, checkerboard_height), checkerboard_grid, CHESSBOARD);
    
    transform_.translation().setZero();
    transform_.matrix().topLeftCorner<3, 3>() = Quaternionf().setIdentity().toRotationMatrix();
    
    // Create subscriptions
    info_sub_ = nh_.subscribe("/camera/rgb/camera_info", 1, &CalibrateKinectCheckerboard::infoCallback, this);
    
    // Also publishers
    pub_ = it_.advertise("calibration_pattern_out", 1);
    detector_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("detector_cloud", 1);
    physical_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("physical_points_cloud", 1);
    
    // Create ideal points
    ideal_points_.push_back( pcl::PointXYZ(0, 0, 0) );
    ideal_points_.push_back( pcl::PointXYZ((checkerboard_width-1)*checkerboard_grid, 0, 0) );
    ideal_points_.push_back( pcl::PointXYZ(0, (checkerboard_height-1)*checkerboard_grid, 0) );
    ideal_points_.push_back( pcl::PointXYZ((checkerboard_width-1)*checkerboard_grid, (checkerboard_height-1)*checkerboard_grid, 0) );
    
    // Create proper gripper tip point
    nh_.param<double>("gripper_tip_x", gripper_tip.point.x, 0.0);
    nh_.param<double>("gripper_tip_y", gripper_tip.point.y, 0.0);
    nh_.param<double>("gripper_tip_z", gripper_tip.point.z, 0.0);
    gripper_tip.header.frame_id = tip_frame;
    
    ROS_INFO("[calibrate] Initialized.");
  }

  void infoCallback(const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    if (calibrated)
      return;
    cam_model_.fromCameraInfo(info_msg);
    pattern_detector_.setCameraMatrices(cam_model_.intrinsicMatrix(), cam_model_.distortionCoeffs());
    
    calibrated = true;
    image_sub_ = nh_.subscribe("/camera/rgb/image_mono", 1, &CalibrateKinectCheckerboard::imageCallback, this);
    
    ROS_INFO("[calibrate] Got image info!");
  }
  
  void pointcloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg)
  {
    sensor_msgs::ImagePtr image_msg(new sensor_msgs::Image);
    pcl::toROSMsg (*msg, *image_msg);
  
    imageCallback(image_msg);
  }
  
  void imageCallback(const sensor_msgs::ImageConstPtr& image_msg)
  {
    try
    {
      input_bridge_ = cv_bridge::toCvCopy(image_msg, "mono8");
      output_bridge_ = cv_bridge::toCvCopy(image_msg, "bgr8");
    }
    catch (cv_bridge::Exception& ex)
    {
      ROS_ERROR("[calibrate] Failed to convert image");
      return;
    }
  
    Eigen::Vector3f translation;
    Eigen::Quaternionf orientation;
    
    if (!pattern_detector_.detectPattern(input_bridge_->image, translation, orientation, output_bridge_->image))
    {
      ROS_INFO("[calibrate] Couldn't detect checkerboard, make sure it's visible in the image.");
      return;
    }
    
    tf::Transform target_transform;
    tf::StampedTransform base_transform;
    try
    {
      ros::Time acquisition_time = image_msg->header.stamp;
      ros::Duration timeout(1.0 / 30.0);
                                   
      target_transform.setOrigin( tf::Vector3(translation.x(), translation.y(), translation.z()) );
      target_transform.setRotation( tf::Quaternion(orientation.x(), orientation.y(), orientation.z(), orientation.w()) );
      tf_broadcaster_.sendTransform(tf::StampedTransform(target_transform, image_msg->header.stamp, image_msg->header.frame_id, target_frame));
    }
    catch (tf::TransformException& ex)
    {
      ROS_WARN("[calibrate] TF exception:\n%s", ex.what());
      return;
    }
    publishCloud(ideal_points_, target_transform, image_msg->header.frame_id);
    
    overlayPoints(ideal_points_, target_transform, output_bridge_);
    
    // Publish calibration image
    pub_.publish(output_bridge_->toImageMsg());
    
    pcl_ros::transformPointCloud(ideal_points_, image_points_, target_transform);
    
    cout << "Got an image callback!" << endl;
    
    calibrate(image_msg->header.frame_id);
    
    ros::shutdown();
  }
  

  void publishCloud(pcl::PointCloud<pcl::PointXYZ> detector_points, tf::Transform &transform, const std::string frame_id)
  {
    // Display to rviz
    pcl::PointCloud<pcl::PointXYZ> transformed_detector_points;
    
    pcl_ros::transformPointCloud(detector_points, transformed_detector_points, transform);
    
    transformed_detector_points.header.frame_id = frame_id;
    detector_pub_.publish(transformed_detector_points);
  }
  
  void overlayPoints(pcl::PointCloud<pcl::PointXYZ> detector_points, tf::Transform &transform, cv_bridge::CvImagePtr& image)
  {  
    // Overlay calibration points on the image
    pcl::PointCloud<pcl::PointXYZ> transformed_detector_points;
    
    pcl_ros::transformPointCloud(detector_points, transformed_detector_points, transform);
    
    int font_face = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
    double font_scale = 1;
    int thickness = 2;
    int radius = 5;
   
    for (unsigned int i=0; i < transformed_detector_points.size(); i++)
    {
      pcl::PointXYZ pt = transformed_detector_points[i];
      cv::Point3d pt_cv(pt.x, pt.y, pt.z);
      cv::Point2d uv;
      uv = cam_model_.project3dToPixel(pt_cv);

      cv::circle(image->image, uv, radius, CV_RGB(255,0,0), -1);
      cv::Size text_size;
      int baseline = 0;
      std::stringstream out;
      out << i+1;
      
      text_size = cv::getTextSize(out.str(), font_face, font_scale, thickness, &baseline);
                            
      cv::Point origin = cvPoint(uv.x - text_size.width / 2,
                               uv.y - radius - baseline/* - thickness*/);
      cv::putText(image->image, out.str(), origin, font_face, font_scale, CV_RGB(255,0,0), thickness);
    }
  }
  
  bool calibrate(const std::string frame_id)
  {
    physical_points_.empty();
    physical_points_.header.frame_id = fixed_frame;
    cout << "Is the checkerboard correct? " << endl;
    cout << "Move edge of gripper to point 1 in image and press Enter. " << endl;
    cin.ignore();
    addPhysicalPoint();
    cout << "Move edge of gripper to point 2 in image and press Enter. " << endl;
    cin.ignore();
    addPhysicalPoint();
    cout << "Move edge of gripper to point 3 in image and press Enter. " << endl;
    cin.ignore();
    addPhysicalPoint();
    cout << "Move edge of gripper to point 4 in image and press Enter. " << endl;
    cin.ignore();
    addPhysicalPoint();
    
    Eigen::Matrix4f t;
    
    physical_pub_.publish(physical_points_);
    
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> svd_estimator;
    svd_estimator.estimateRigidTransformation( physical_points_, image_points_, t );

    // Output       
    tf::Transform transform = tfFromEigen(t), trans_full, camera_transform_unstamped;
    tf::StampedTransform camera_transform;
  
    cout << "Resulting transform (camera frame -> fixed frame): " << endl << t << endl << endl;
    
    try
    {
      tf_listener_.lookupTransform(frame_id, camera_frame, ros::Time(0), camera_transform);
    }
    catch (tf::TransformException& ex)
    {
      ROS_WARN("[calibrate] TF exception:\n%s", ex.what());
      return false;
    }

    camera_transform_unstamped = camera_transform;
    trans_full = camera_transform_unstamped.inverse()*transform;
    
    Eigen::Matrix4f t_full = EigenFromTF(trans_full);
    Eigen::Matrix4f t_full_inv = (Eigen::Transform<float,3,Affine>(t_full).inverse()).matrix();
    
    cout << "Resulting transform (fixed frame -> camera frame): " << endl << t_full << endl << endl;
    printStaticTransform(t_full_inv, fixed_frame, camera_frame);

    return true;
  }
  
  void printStaticTransform(Eigen::Matrix4f& transform, const std::string frame1, const std::string frame2)
  {
    Eigen::Quaternionf quat(transform.topLeftCorner<3,3>() );
    Eigen::Vector3f translation(transform.topRightCorner<3,1>() );
    
    cout << "Static transform publisher (use for external kinect): " << endl;
    
    cout << "rosrun tf static_transform_publisher x y z qx qy qz qw frame_id child_frame_id period_in_ms" << endl;
    cout << "rosrun tf static_transform_publisher " << translation.x() << " "
         << translation.y() << " " << translation.z() << " " 
         << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << " "
         << frame1 << " " << frame2 << " 100" << endl << endl;
         
    tf::Transform temp_tf_trans = tfFromEigen(transform);
    
    double yaw, pitch, roll;
    
    std::string fixed_frame_urdf(fixed_frame);
    
    // If there's a leading '/' character, remove it, as xacro can't deal with 
    // extra characters in the link name.
    if (fixed_frame_urdf.size() > 0 && fixed_frame_urdf[0] == '/')
      fixed_frame_urdf.erase(0, 1);
    
    temp_tf_trans.getBasis().getEulerYPR(yaw, pitch, roll);
    
    cout << "URDF output (use for kinect on robot): " << endl;
    
    cout << "<?xml version=\"1.0\"?>\n<robot>\n" << 
          "\t<property name=\"turtlebot_calib_cam_x\" value=\"" << translation.x() << "\" />\n" <<
          "\t<property name=\"turtlebot_calib_cam_y\" value=\"" << translation.y() << "\" />\n" <<
          "\t<property name=\"turtlebot_calib_cam_z\" value=\"" << translation.z() << "\" />\n" <<
          "\t<property name=\"turtlebot_calib_cam_rr\" value=\"" << roll << "\" />\n" <<
          "\t<property name=\"turtlebot_calib_cam_rp\" value=\"" << pitch << "\" />\n" <<
          "\t<property name=\"turtlebot_calib_cam_ry\" value=\"" << yaw << "\" />\n" <<
          "\t<property name=\"turtlebot_kinect_frame_name\" value=\"" << fixed_frame_urdf << "\" />\n" <<
          "</robot>" << endl << endl;
  }
  
  void addPhysicalPoint()
  {
    geometry_msgs::PointStamped pt_out;
    
    try
    {
      tf_listener_.transformPoint(fixed_frame, gripper_tip, pt_out);
    }
    catch (tf::TransformException& ex)
    {
      ROS_WARN("[calibrate] TF exception:\n%s", ex.what());
      return;
    }
    
    physical_points_.push_back(pcl::PointXYZ(pt_out.point.x, pt_out.point.y, pt_out.point.z));
  }

  void convertIdealPointstoPointcloud()
  {
    detector_points_.points.resize(pattern_detector_.ideal_points.size());
    for (unsigned int i=0; i < pattern_detector_.ideal_points.size(); i++)
    {
      cv::Point3f pt = pattern_detector_.ideal_points[i];
      detector_points_[i].x = pt.x; detector_points_[i].y = pt.y; detector_points_[i].z = pt.z; 
    }
  }
  
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "calibrate_kinect_arm");
  
  CalibrateKinectCheckerboard cal;
  ros::spin();
}

