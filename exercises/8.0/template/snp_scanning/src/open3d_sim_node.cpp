#include <boost/filesystem.hpp>
#include <rclcpp/rclcpp.hpp>
#include <industrial_reconstruction_msgs/srv/start_reconstruction.hpp>
#include <industrial_reconstruction_msgs/srv/stop_reconstruction.hpp>
#include <visualization_msgs/msg/marker.hpp>

static const std::string MESH_FILE_PARAMETER = "mesh_file";
static const std::string REFERENCE_FRAME_PARAMETER = "reference_frame";
static const std::string MESH_TOPIC = "mesh";

class Open3dSimServer : public rclcpp::Node
{
public:
  explicit Open3dSimServer(const std::string& name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node(name, options)
  {
    using namespace std::placeholders;

    start_srv_ = create_service<industrial_reconstruction_msgs::srv::StartReconstruction>(
        "start_reconstruction", std::bind(&Open3dSimServer::startCB, this, _1, _2));

    stop_srv_ = create_service<industrial_reconstruction_msgs::srv::StopReconstruction>(
        "stop_reconstruction", std::bind(&Open3dSimServer::stopCB, this, _1, _2));

    scan_mesh_pub_ = create_publisher<visualization_msgs::msg::Marker>(MESH_TOPIC, 10);

    declare_parameter<std::string>(MESH_FILE_PARAMETER, "");
    declare_parameter<std::string>(REFERENCE_FRAME_PARAMETER, "");
  }

private:
  void startCB(const std::shared_ptr<industrial_reconstruction_msgs::srv::StartReconstruction::Request> /*request*/,
               std::shared_ptr<industrial_reconstruction_msgs::srv::StartReconstruction::Response> response)
  {
    response->success = true;
    RCLCPP_INFO(get_logger(), " Sim Started");
  }

  void stopCB(const std::shared_ptr<industrial_reconstruction_msgs::srv::StopReconstruction::Request> request,
              std::shared_ptr<industrial_reconstruction_msgs::srv::StopReconstruction::Response> response)
  {
    try
    {
      // get parameter named "mesh_sourcepath" and local-var name it filepath_str
      std::string mesh_file;
      if (!get_parameter(MESH_FILE_PARAMETER, mesh_file))
        throw std::runtime_error("Failed to get parameter '" + MESH_FILE_PARAMETER + "'");

      std::string reference_frame;
      if (!get_parameter(REFERENCE_FRAME_PARAMETER, reference_frame))
        throw std::runtime_error("Failed to get parameter '" + REFERENCE_FRAME_PARAMETER + "'");

      boost::filesystem::path mesh_source_path(mesh_file);
      boost::filesystem::path mesh_target_path(request->mesh_filepath);
      boost::filesystem::copy_file(mesh_source_path, mesh_target_path,
                                   boost::filesystem::copy_option::overwrite_if_exists);

      // Publish the mesh
      {
        visualization_msgs::msg::Marker mesh_marker;
        mesh_marker.header.frame_id = reference_frame;

        mesh_marker.color.r = 200;
        mesh_marker.color.g = 200;
        mesh_marker.color.b = 0;
        mesh_marker.color.a = 1;

        mesh_marker.scale.x = 1;
        mesh_marker.scale.y = 1;
        mesh_marker.scale.z = 1;

        mesh_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        mesh_marker.mesh_resource = "file://" + mesh_file;

        scan_mesh_pub_->publish(mesh_marker);
      }

      response->success = true;
      RCLCPP_INFO_STREAM(get_logger(),
                         "Scanning simulation complete; mesh saved to '" << request->mesh_filepath << "'");
    }
    catch (const std::exception& ex)
    {
      response->success = false;
      RCLCPP_ERROR_STREAM(get_logger(), "Scanning simulation failed: '" << ex.what() << "'");
    }
  }

  rclcpp::Service<industrial_reconstruction_msgs::srv::StartReconstruction>::SharedPtr start_srv_;
  rclcpp::Service<industrial_reconstruction_msgs::srv::StopReconstruction>::SharedPtr stop_srv_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr scan_mesh_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Open3dSimServer>("open3d_sim_server");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
