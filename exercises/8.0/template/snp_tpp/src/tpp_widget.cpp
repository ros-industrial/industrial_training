#include <snp_tpp/tpp_widget.h>
#include "ui_tpp_widget.h"

#include <noether_gui/widgets/tpp_pipeline_widget.h>
#include <noether_gui/utils.h>
#include <pcl/io/vtk_lib_io.h>
#include <QFileDialog>
#include <QMessageBox>
#include <QStandardPaths>
#include <QTextStream>
#include <tf2_eigen/tf2_eigen.h>
#include <yaml-cpp/yaml.h>

namespace
{
geometry_msgs::msg::PoseArray toMsg(const noether::ToolPathSegment& segment)
{
  geometry_msgs::msg::PoseArray pose_array;
  pose_array.poses.reserve(segment.size());
  for (auto waypoint : segment)
  {
    // Renormalize orientation
    Eigen::Quaterniond q(waypoint.linear());
    q.normalize();
    waypoint.matrix().block<3, 3>(0, 0) = q.toRotationMatrix();

    pose_array.poses.push_back(tf2::toMsg(waypoint));
  }
  return pose_array;
}

snp_msgs::msg::ToolPath toMsg(const noether::ToolPath& path)
{
  snp_msgs::msg::ToolPath path_msg;
  path_msg.segments.reserve(path.size());
  for (const auto& segment : path)
    path_msg.segments.push_back(toMsg(segment));
  return path_msg;
}

snp_msgs::msg::ToolPaths toMsg(const noether::ToolPaths& paths)
{
  snp_msgs::msg::ToolPaths paths_msg;
  paths_msg.paths.reserve(paths.size());
  for (const auto& path : paths)
    paths_msg.paths.push_back(toMsg(path));
  return paths_msg;
}

}  // namespace

namespace snp_tpp
{
TPPWidget::TPPWidget(rclcpp::Node::SharedPtr node, boost_plugin_loader::PluginLoader&& loader, QWidget* parent)
  : QWidget(parent), ui_(new Ui::TPPWidget())
{
  // Configure the UI
  ui_->setupUi(this);

  pipeline_widget_ = new noether::TPPPipelineWidget(std::move(loader), this);
  ui_->verticalLayout->addWidget(pipeline_widget_);

  connect(ui_->push_button_load_configuration, &QPushButton::clicked, this, &TPPWidget::onLoadConfiguration);
  connect(ui_->push_button_save_configuration, &QPushButton::clicked, this, &TPPWidget::onSaveConfiguration);

  // Set up the ROS interfaces
  server_ = node->create_service<snp_msgs::srv::GenerateToolPaths>(
      "/generate_tool_paths", std::bind(&TPPWidget::callback, this, std::placeholders::_1, std::placeholders::_2));

  // Load a parameter-specified configuration file for the tool path planner
  std::string config_file;
  node->declare_parameter("config_file", config_file);
  node->get_parameter("config_file", config_file);
  if (!config_file.empty())
    configureTPPPipeline(config_file);
}

void TPPWidget::configureTPPPipeline(const std::string& file)
{
  try
  {
    pipeline_widget_->configure(YAML::LoadFile(file));
    ui_->line_edit_configuration->setText(QString::fromStdString(file));
  }
  catch (const YAML::BadFile&)
  {
    std::stringstream ss;
    ss << "Failed to open YAML file at '" << file << "'";
    QMessageBox::warning(this, "Configuration Error", QString::fromStdString(ss.str()));
  }
  catch (const std::exception& ex)
  {
    std::stringstream ss;
    noether::printException(ex, ss);
    QMessageBox::warning(this, "Configuration Error", QString::fromStdString(ss.str()));
  }
}

void TPPWidget::onLoadConfiguration(const bool /*checked*/)
{
  QString file = ui_->line_edit_configuration->text();
  if (file.isEmpty())
    file = QStandardPaths::standardLocations(QStandardPaths::HomeLocation).at(0);

  file = QFileDialog::getOpenFileName(this, "Load configuration file", file, "YAML files (*.yaml)");
  if (file.isEmpty())
    return;

  configureTPPPipeline(file.toStdString());
}

void TPPWidget::onSaveConfiguration(const bool /*checked*/)
{
  try
  {
    QString file = ui_->line_edit_configuration->text();
    if (file.isEmpty())
      file = QStandardPaths::standardLocations(QStandardPaths::HomeLocation).at(0);

    file = QFileDialog::getSaveFileName(this, "Save configuration file", file, "YAML files (*.yaml)");
    if (file.isEmpty())
      return;

    YAML::Node config;
    pipeline_widget_->save(config);

    std::ofstream ofh(file.toStdString());
    if (!ofh)
      throw std::runtime_error("Failed to open output file at '" + file.toStdString() + "'");

    ofh << config;
    QMessageBox::information(this, "Configuration", "Successfully saved tool path planning pipeline configuration");
  }
  catch (const std::exception& ex)
  {
    std::stringstream ss;
    noether::printException(ex, ss);
    QMessageBox::warning(this, "Save Error", QString::fromStdString(ss.str()));
  }
}

void TPPWidget::callback(const snp_msgs::srv::GenerateToolPaths::Request::SharedPtr req,
                         const snp_msgs::srv::GenerateToolPaths::Response::SharedPtr res)
{
  try
  {
    noether::ToolPathPlannerPipeline pipeline = pipeline_widget_->createPipeline();

    // Load the mesh
    pcl::PolygonMesh mesh;
    if (pcl::io::loadPolygonFile(req->mesh_filename, mesh) < 1)
      throw std::runtime_error("Failed to load mesh");
    mesh.header.frame_id = req->mesh_frame;

    // Plan the tool paths
    std::vector<noether::ToolPaths> tool_paths = pipeline.plan(mesh);

    res->tool_paths = toMsg(tool_paths.at(0));
    res->success = true;
    res->message = "Success";
  }
  catch (const std::exception& ex)
  {
    res->message = ex.what();
    res->success = false;
  }
}

}  // namespace snp_tpp
