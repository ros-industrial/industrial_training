#pragma once

#include <boost_plugin_loader/plugin_loader.h>
#include <QWidget>
#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>
#include <snp_msgs/srv/generate_tool_paths.hpp>

namespace noether
{
class TPPPipelineWidget;
}

namespace Ui
{
class TPPWidget;
}

namespace snp_tpp
{
class TPPWidget : public QWidget
{
  Q_OBJECT

public:
  TPPWidget(rclcpp::Node::SharedPtr node, boost_plugin_loader::PluginLoader&& loader, QWidget* parent = nullptr);

private:
  void callback(const snp_msgs::srv::GenerateToolPaths::Request::SharedPtr req,
                const snp_msgs::srv::GenerateToolPaths::Response::SharedPtr res);

  void configureTPPPipeline(const std::string& file);
  void onLoadConfiguration(const bool /*checked*/);
  void onSaveConfiguration(const bool /*checked*/);

  noether::TPPPipelineWidget* pipeline_widget_{ nullptr };
  rclcpp::Service<snp_msgs::srv::GenerateToolPaths>::SharedPtr server_{ nullptr };
  Ui::TPPWidget* ui_{ nullptr };
};

}  // namespace snp_tpp
