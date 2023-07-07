#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

class ExecSimServer : public rclcpp::Node
{
public:
  explicit ExecSimServer(const std::string& name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node(name, options)
  {
    using namespace std::placeholders;

    enable_srv_ =
        create_service<std_srvs::srv::Trigger>("robot_enable", std::bind(&ExecSimServer::enableCallback, this, _1, _2));

    disable_srv_ = create_service<std_srvs::srv::Trigger>("robot_disable",
                                                          std::bind(&ExecSimServer::disableCallback, this, _1, _2));

    RCLCPP_INFO(get_logger(), "Started simulated robot enable node");
  }

private:
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_srv_;

  void enableCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    callback(request, response, "Enabled robot");
  }

  void disableCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    callback(request, response, "Disabled robot");
  }

  void callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response, const std::string& message)
  {
    response->success = true;
    response->message = message;
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ExecSimServer>("robot_enable_server_sim");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
