#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/make_shared.hpp>

#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>

#include <pcl_ros/transforms.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>

#include <Eigen/Geometry>

// ros parameter names
static const std::string CLOUD_DESCRIPTION_FILE = "obstacles_cloud_description_file";

// yaml fields
static const std::string CLOUD_DESCRIPTIONS = "point_cloud_descriptions";
static const std::string FRAME_ID = "frame_id";

// topics
const std::string POINT_CLOUD_TOPIC = "generated_cloud";

template <class P>
static P loadParameter(rclcpp::Node::SharedPtr n, const std::string& param_name)
{
  P param_val;
  if (!n->get_parameter(param_name, param_val))
  {
    throw std::runtime_error(boost::str(boost::format("Failed to load the \"%s\" parameter") % param_name));
  }

  return param_val;
}

class GeneratePointCloud
{
  typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
  struct Description
  {
    Eigen::Vector3d size;
    Eigen::Isometry3d transform;
    double resolution;
  };

public:
  GeneratePointCloud(rclcpp::Node::SharedPtr node) : node_(node) {}

  void init()
  {
    namespace fs = boost::filesystem;

    // loading parameters
    const std::string cloud_yaml_file = loadParameter<std::string>(node_, CLOUD_DESCRIPTION_FILE);

    // check yaml file
    if (!fs::exists(fs::path(cloud_yaml_file)))
    {
      throw std::runtime_error(boost::str(boost::format("The \"%s\" file does not exists") % cloud_yaml_file));
    }

    // load yaml now
    try
    {
      YAML::Node root_node = YAML::LoadFile(cloud_yaml_file);
      YAML::Node cloud_desc_node = root_node[CLOUD_DESCRIPTIONS];
      frame_id_ = root_node[FRAME_ID].as<std::string>();

      // parameter numeric_fiels
      std::map<std::string, double> numeric_fields;
      numeric_fields.insert(std::make_pair("x", 0));
      numeric_fields.insert(std::make_pair("y", 0));
      numeric_fields.insert(std::make_pair("z", 0));
      numeric_fields.insert(std::make_pair("rx", 0));
      numeric_fields.insert(std::make_pair("ry", 0));
      numeric_fields.insert(std::make_pair("rz", 0));
      numeric_fields.insert(std::make_pair("l", 0));
      numeric_fields.insert(std::make_pair("w", 0));
      numeric_fields.insert(std::make_pair("h", 0));
      numeric_fields.insert(std::make_pair("resolution", 0));

      // parsing each entry
      for (std::size_t i = 0; i < cloud_desc_node.size(); i++)
      {
        YAML::Node entry_node = cloud_desc_node[i];

        // parse numeric fields
        for (decltype(numeric_fields)::iterator i = numeric_fields.begin(); i != numeric_fields.end(); i++)
        {
          if (entry_node[i->first])
          {
            double val = entry_node[i->first].as<double>();
            numeric_fields[i->first] = val;
          }
          else
          {
            std::string err_msg =
                boost::str(boost::format("Point Cloud description entry is missing field \"%s\"") % i->first);
            throw std::runtime_error(err_msg);
          }
        }

        // populating structure
        Description d;
        d.size = Eigen::Vector3d(numeric_fields["l"], numeric_fields["w"], numeric_fields["h"]);
        Eigen::Quaterniond q = Eigen::AngleAxisd(numeric_fields["rx"], Eigen::Vector3d::UnitX()) *
                               Eigen::AngleAxisd(numeric_fields["ry"], Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(numeric_fields["rz"], Eigen::Vector3d::UnitZ());
        d.transform = Eigen::Translation3d(numeric_fields["x"], numeric_fields["y"], numeric_fields["z"]) * q;
        d.resolution = numeric_fields["resolution"];

        cloud_descriptions_.push_back(d);
      }
    }
    catch (YAML::Exception& e)
    {
      throw std::runtime_error(boost::str(boost::format("Failed to parse yaml file \"%s\"") % cloud_yaml_file));
    }
  }

  void run()
  {
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher =
        node_->create_publisher<sensor_msgs::msg::PointCloud2>(POINT_CLOUD_TOPIC, 1);

    // initializing
    init();

    // generating data and running main loop
    genenerate_cloud();

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(full_cloud_, msg);

    rclcpp::Duration loop_duration(std::chrono::duration<double>(0.4));
    while (rclcpp::ok())
    {
      msg.header.stamp = node_->get_clock()->now() - loop_duration;
      cloud_publisher->publish(msg);
      rclcpp::sleep_for(loop_duration.to_chrono<std::chrono::nanoseconds>());
    }
  }

protected:
  void genenerate_cloud()
  {
    full_cloud_.clear();
    for (unsigned int i = 0; i < cloud_descriptions_.size(); i++)
    {
      Cloud box;
      Description& desc = cloud_descriptions_[i];
      createBox(desc, box);

      // transforming box
      pcl::transformPointCloud(box, box, desc.transform.cast<float>());

      // concatenating box
      full_cloud_ += box;
    }

    full_cloud_.header.frame_id = frame_id_;
  }

  void createRectangularPatch(Eigen::Vector3d start, Eigen::Vector3d end, double res, Cloud& patch)
  {
    int count_x = (end.x() - start.x()) / res + 1;
    int count_y = (end.y() - start.y()) / res + 1;

    pcl::PointXYZ p;
    p.z = 0;
    patch.resize(count_x * count_y);
    int counter = 0;
    for (int i = 0; i < count_x; i++)
    {
      p.x = start.x() + res * i;
      for (int j = 0; j < count_y; j++)
      {
        p.y = start.y() + res * j;
        patch.points[counter] = p;
        counter++;
      }
    }
  }

  void createBox(const Description& desc, Cloud& box_points)
  {
    // face points
    Cloud top, bottom, front, rear, left, right, temp;
    Eigen::Vector3d start, end;

    // transforms
    Eigen::Isometry3d t;
    // Eigen::Affine3d eigen3d;

    // ================================ create top and bottom patches ===============================
    start = Eigen::Vector3d(-desc.size.x() / 2, -desc.size.y() / 2, 0);
    end = Eigen::Vector3d(desc.size.x() / 2, desc.size.y() / 2, 0);
    createRectangularPatch(start, end, desc.resolution, temp);

    // transform cloud to top
    t = Eigen::Translation3d(0, 0, 0.5f * desc.size.z());
    // t = tf2::Transform(Eigen::Quaterniond::getIdentity(),Eigen::Vector3d(0,0,0.5f*desc.size.z()));
    // tf2::transformTFToEigen(t,eigen3d);
    // pcl::transformPointCloud(temp,top,Eigen::Affine3f(eigen3d));
    pcl::transformPointCloud(temp, top, t.cast<float>());

    // transform cloud to bottom
    t = Eigen::Translation3d(0, 0, 0.5f * -desc.size.z());
    // t = tf2::Transform(Eigen::Quaterniond::getIdentity(),Eigen::Vector3d(0, 0, 0.5f*-desc.size.z()));
    // tf2::transformTFToEigen(t,eigen3d);
    // pcl::transformPointCloud(temp,bottom,Eigen::Affine3f(eigen3d));
    pcl::transformPointCloud(temp, bottom, t.cast<float>());

    // concatenate
    box_points += top;
    box_points += bottom;

    // ================================ create front and rear patches ===============================
    start = Eigen::Vector3d(-desc.size.x() / 2, -desc.size.z() / 2, 0);
    end = Eigen::Vector3d(desc.size.x() / 2, desc.size.z() / 2, 0);
    createRectangularPatch(start, end, desc.resolution, temp);

    // transform cloud to front
    t = Eigen::Translation3d(0, 0.5f * desc.size.y(), 0) * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX());
    /*			t = tf2::Transform(Eigen::Quaterniond(Eigen::Vector3d(1,0,0),M_PI_2)
            ,Eigen::Vector3d(0,0.5f*desc.size.y(),0));
          tf2::transformTFToEigen(t,eigen3d);
          pcl::transformPointCloud(temp,front,Eigen::Affine3f(eigen3d));*/
    pcl::transformPointCloud(temp, front, t.cast<float>());

    // transform cloud to rear
    t = Eigen::Translation3d(0, 0.5f * -desc.size.y(), 0) * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX());
    /*			t = tf2::Transform(Eigen::Quaterniond(Eigen::Vector3d(1,0,0),M_PI_2)
                  ,Eigen::Vector3d(0,0.5f*-desc.size.y(),0));
          tf2::transformTFToEigen(t,eigen3d);
          pcl::transformPointCloud(temp,rear,Eigen::Affine3f(eigen3d));*/
    pcl::transformPointCloud(temp, rear, t.cast<float>());

    box_points += front;
    box_points += rear;

    // ================================ create left and right patches ===============================
    start = Eigen::Vector3d(-desc.size.z() / 2, -desc.size.y() / 2, 0);
    end = Eigen::Vector3d(desc.size.z() / 2, desc.size.y() / 2, 0);
    createRectangularPatch(start, end, desc.resolution, temp);

    // transform cloud to left
    t = Eigen::Translation3d(0.5f * desc.size.x(), 0, 0) * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY());
    /*			t = tf2::Transform(Eigen::Quaterniond(Eigen::Vector3d(0,1,0),M_PI_2)
            ,Eigen::Vector3d(0.5f*desc.size.x(),0,0));
          tf2::transformTFToEigen(t,eigen3d);
          pcl::transformPointCloud(temp,left,Eigen::Affine3f(eigen3d));*/
    pcl::transformPointCloud(temp, left, t.cast<float>());

    // transform cloud to right
    t = Eigen::Translation3d(0.5f * -desc.size.x(), 0, 0) * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY());
    /*			t = tf2::Transform(Eigen::Quaterniond(Eigen::Vector3d(0,1,0),M_PI_2)
            ,Eigen::Vector3d(0.5f*-desc.size.x(),0,0));
          tf2::transformTFToEigen(t,eigen3d);
          pcl::transformPointCloud(temp,right,Eigen::Affine3f(eigen3d));*/
    pcl::transformPointCloud(temp, right, t.cast<float>());

    box_points += left;
    box_points += right;
  }

protected:
  rclcpp::Node::SharedPtr node_;
  std::vector<Description> cloud_descriptions_;
  std::string frame_id_;
  Cloud full_cloud_;
  float resolution_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("generate_point_cloud_node", node_options);

  // spinning node in a separate thread
  std::thread spin_thread([node]() {
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  });

  GeneratePointCloud g(node);
  g.run();

  spin_thread.join();
  rclcpp::shutdown();

  return 0;
}
