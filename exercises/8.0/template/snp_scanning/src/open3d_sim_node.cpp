#include <boost/filesystem.hpp>
#include <rclcpp/rclcpp.hpp>
#include <industrial_reconstruction_msgs/srv/start_reconstruction.hpp>
#include <industrial_reconstruction_msgs/srv/stop_reconstruction.hpp>

static const std::string MESH_FILE_PARAMETER = "mesh_file";

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

    declare_parameter<std::string>(MESH_FILE_PARAMETER, "");
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

      boost::filesystem::path mesh_source_path(mesh_file);
      boost::filesystem::path mesh_target_path(request->mesh_filepath);
      boost::filesystem::copy_file(mesh_source_path, mesh_target_path,
                                   boost::filesystem::copy_option::overwrite_if_exists);
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
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Open3dSimServer>("open3d_sim_server");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
