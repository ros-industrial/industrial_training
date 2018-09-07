#include <urdf/model.h>
#include "ros/ros.h"
#include "ros/package.h"

#include "gl_depth_sim/sim_depth_camera.h"
#include "gl_depth_sim/mesh_loader.h"
#include "gl_depth_sim/interfaces/pcl_interface.h"

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <chrono>

static Eigen::Affine3d lookat(const Eigen::Vector3d& origin, const Eigen::Vector3d& eye, const Eigen::Vector3d& up)
{
  Eigen::Vector3d z = (eye - origin).normalized();   // get unit vector in direction you want to look
  Eigen::Vector3d x = z.cross(up).normalized();      // cross to get x and y
  Eigen::Vector3d y = z.cross(x).normalized();

  auto p = Eigen::Affine3d::Identity();
  p.translation() = origin;
  p.matrix().col(0).head<3>() = x;
  p.matrix().col(1).head<3>() = y;
  p.matrix().col(2).head<3>() = z;
  return p;
}

static std::string getfullpath(const std::string relativePath)
{
  std::string fullPath;
if (relativePath.substr(0,10) == "package://")
{
  unsigned last = relativePath.substr(10,relativePath.back()).find("/");
  std::string pkgPath = relativePath.substr (10,last);
  fullPath = ros::package::getPath(pkgPath) + relativePath.substr(10+last,relativePath.back());
  return fullPath;
}
else
{
  return "Error: converts 'package://' to full package path";
}
}



int main(int argc, char** argv){
  ros::init(argc, argv, "ros_depth_sim_orbit");
  ros::NodeHandle nh, pnh ("~");

  // Setup ROS interfaces
  ros::Publisher cloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/camera/depth_registered/points", 1, true);

//  tf::TransformBroadcaster broadcaster;
  tf::TransformListener listener;


  std::string urdf_file = pnh.param<std::string>("urdf_path" , ros::package::getPath("pick_and_place_support") + "/urdf/pick_and_place.urdf" );
  std::string base_frame = pnh.param<std::string>("base_frame", "world");
  std::string camera_frame = pnh.param<std::string>("camera_frame", "/sim_camera_link");

  double radius = pnh.param<double>("radius", 1.0);
  double z = pnh.param<double>("z", 1.0);

  double focal_length = pnh.param<double>("focal_length", 550.0);
  int width = pnh.param<int>("width", 640);
  int height = pnh.param<int>("height", 480);



  gl_depth_sim::CameraProperties props;
  props.width = width;
  props.height = height;
  props.fx = focal_length;
  props.fy = focal_length;
  props.cx = props.width / 2;
  props.cy = props.height / 2;
  props.z_near = 0.25;
  props.z_far = 10.0f;

  // Create the simulation
  gl_depth_sim::SimDepthCamera sim (props);

  // ---------------- Parse URDF ----------------------
  urdf::Model model;
  if (!model.initFile(urdf_file)){
    ROS_ERROR("Failed to parse urdf file");
    return -1;
  }
  ROS_INFO("Successfully parsed urdf file");

  auto linkMap = model.links_;  //Returns a map of links
  std::cout << "Links Found:" << std::endl;
  std::map<std::string, urdf::LinkSharedPtr>::iterator it2 = linkMap.begin();

  for(it2; it2!=linkMap.end(); it2++)
  {
    std::cout<<it2->first<<" :: "<<it2->second<< "   ";
    auto linkPtr =  model.getLink(it2->first);

    // Exclude links with no geometry
    if ((linkPtr && linkPtr->visual && linkPtr->visual->geometry) && (linkPtr && linkPtr->visual))
    {
      auto tmp =  boost::dynamic_pointer_cast<const urdf::Mesh>(linkPtr->visual->geometry);
      auto tmp2 = linkPtr->visual;
      if (tmp && tmp2){


        //Get filepath and link name
        std::string filepath =  tmp->filename;
        std::string linkName = linkPtr->name;        // This is also the name of the tf associated with this link

//        std::cout << filepath  << linkPtr->parent_joint->name;
//        std::cout<< getfullpath(filepath);


        //Exclude unsupported filetypes
        //dae files are still broken as of 8/21/18. The internal transforms are not properly imported by assimp in gl_depth_sim
        if (filepath.substr(filepath.size()-3) == "STL" ||  filepath.substr(filepath.size()-3) == "stl" ){
          auto mesh_ptr = gl_depth_sim::loadMesh(getfullpath(filepath));
          if (!mesh_ptr)
          {
            ROS_ERROR_STREAM("Unable to load mesh from path: " << filepath);
            ROS_ERROR_STREAM("Note: Only stl or STL are supported at this time");
            return 1;
          }
          else{
            ROS_INFO_STREAM("Loading mesh from: " << getfullpath(filepath));;
          }
          auto mesh_loc = Eigen::Affine3d::Identity();
          tf::StampedTransform meshTransform;
          listener.waitForTransform("/world","/" + linkName, ros::Time(0), ros::Duration(5));
          listener.lookupTransform("/world", "/" + linkName, ros::Time(0), meshTransform);
          tf::transformTFToEigen(meshTransform,mesh_loc);
          sim.add(linkName, *mesh_ptr, mesh_loc);
        }

      }}
    std::cout << std::endl;
  }

  // mostly copy pasted from "Creating a Cube Mesh in Unity3d"
  std::vector<unsigned int> indices =
  {
    0, 2, 1,
    0, 3, 2,
    2, 3, 4,
    2, 4, 5,
    1, 2, 5,
    1, 5, 6,
    0, 7, 4,
    0, 4, 3,
    5, 4, 7,
    5, 7, 6,
    0, 6, 7,
    0, 1, 6
  };
  gl_depth_sim::EigenAlignedVec<Eigen::Vector3f> vertices =
  {
    Eigen::Vector3f(-0.5, -0.5, -0.5),
    Eigen::Vector3f(0.5, -0.5, -0.5),
    Eigen::Vector3f(0.5, 0.5, -0.5),
    Eigen::Vector3f(-0.5, 0.5, -0.5),
    Eigen::Vector3f(-0.5, 0.5, 0.5),
    Eigen::Vector3f(0.5, 0.5, 0.5),
    Eigen::Vector3f(0.5, -0.5, 0.5),
    Eigen::Vector3f(-0.5, -0.5, 0.5),
  };

  double box_side;
  nh.getParam("box_side", box_side);
  for (std::size_t i = 0; i < vertices.size(); i++)
  {
       vertices[i] *= box_side;
  }
  gl_depth_sim::Mesh box_mesh(vertices, indices);

  std::string box_parent_link;
  double box_x, box_y;
  nh.getParam("box_parent_link", box_parent_link);
  nh.getParam("box_x", box_x);
  nh.getParam("box_y", box_y);

  tf::StampedTransform world_to_box_tf;
  Eigen::Affine3d world_to_box;
  listener.lookupTransform("world", box_parent_link, ros::Time(0.0), world_to_box_tf);
  tf::transformTFToEigen(world_to_box_tf, world_to_box);
  world_to_box.translation() += Eigen::Vector3d(box_x, box_y, box_side/2.0);
  sim.add("box_1", box_mesh, world_to_box);



  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.header.frame_id = camera_frame;

  while (ros::ok())
  {

    //Step 1: Get camera tranform and render point cloud
    tf::StampedTransform camera_transform;
    try{
      listener.lookupTransform("/world", "/sim_camera_link", ros::Time(0), camera_transform);
    }
    catch (tf::TransformException &ex) {
//      ROS_ERROR("%s",ex.what());
      continue;
    }
    auto pose = Eigen::Affine3d::Identity();
    tf::transformTFToEigen(camera_transform, pose);
//    pose.rotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0,1,0))); // currently done in launch file with seperate TF
    const auto depth_img = sim.render(pose);

    // Step 2: Publish the cloud
    gl_depth_sim::toPointCloudXYZ(props, depth_img, cloud);
    pcl_conversions::toPCL(ros::Time::now(), cloud.header.stamp);
    cloud_pub.publish(cloud);


    ros::spinOnce();
  }

  return 0;
}
