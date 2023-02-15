#include "tpp_widget.h"

#include <noether_gui/widgets/tpp_pipeline_widget.h>
#include <pcl/io/vtk_lib_io.h>
#include <QVBoxLayout>
#include <QScrollArea>
#include <tf2_eigen/tf2_eigen.h>

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
TPPWidget::TPPWidget(rclcpp::Node::SharedPtr node, QWidget* parent) : QWidget(parent)
{
  server_ = node->create_service<snp_msgs::srv::GenerateToolPaths>(
      "/generate_tool_paths", std::bind(&TPPWidget::callback, this, std::placeholders::_1, std::placeholders::_2));

  boost_plugin_loader::PluginLoader loader;
  loader.search_libraries.insert(NOETHER_GUI_PLUGINS);
  loader.search_libraries.insert(SNP_TPP_GUI_PLUGINS);

  pipeline_widget_ = new noether::TPPPipelineWidget(std::move(loader), this);

  auto scroll_area = new QScrollArea(this);
  scroll_area->setWidget(pipeline_widget_);
  scroll_area->setVerticalScrollBarPolicy(Qt::ScrollBarPolicy::ScrollBarAsNeeded);
  scroll_area->setHorizontalScrollBarPolicy(Qt::ScrollBarPolicy::ScrollBarAlwaysOff);

  QSizePolicy size_policy(QSizePolicy::MinimumExpanding, QSizePolicy::Expanding);
  size_policy.setHorizontalStretch(0);
  size_policy.setVerticalStretch(0);
  scroll_area->setSizePolicy(size_policy);
  scroll_area->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  scroll_area->setWidgetResizable(true);

  auto layout = new QVBoxLayout(this);
  layout->addWidget(scroll_area);
  setLayout(layout);
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
