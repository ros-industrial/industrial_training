#include <snp_tpp/roi_selection_mesh_modifier_widget.h>
#include <snp_tpp/roi_selection_mesh_modifier.h>
#include "ui_roi_selection_mesh_modifier_widget.h"

#include <noether_gui/plugin_interface.h>
#include <noether_gui/utils.h>

static const std::string MAX_CLUSTER_SIZE_KEY = "max_cluster_size";
static const std::string MIN_CLUSTER_SIZE_KEY = "min_cluster_size";
static const std::string CLUSTER_TOLERANCE_KEY = "cluster_tolerance";
static const std::string PLANE_DISTANCE_THRESHOLD_KEY = "plane_distance_threshold";

namespace snp_tpp
{
ROISelectionMeshModifierWidget::ROISelectionMeshModifierWidget(QWidget* parent)
  : noether::MeshModifierWidget(parent)
  , ui_(new Ui::ROISelectionMeshModifier())
  , node_(std::make_shared<rclcpp::Node>("roi_selection_mesh_modifier"))
  , client_(node_->create_client<rviz_polygon_selection_tool::srv::GetSelection>("get_selection"))
  , thread_(std::bind(&ROISelectionMeshModifierWidget::spin, this))
{
  ui_->setupUi(this);
}

ROISelectionMeshModifierWidget::~ROISelectionMeshModifierWidget()
{
  executor_.cancel();
  thread_.join();
}

void ROISelectionMeshModifierWidget::spin()
{
  executor_.add_node(node_);
  executor_.spin();
}

noether::MeshModifier::ConstPtr ROISelectionMeshModifierWidget::create() const
{
  noether::ExtrudedPolygonSubMeshExtractor extractor;
  extractor.extractor.params.max_cluster_size = ui_->max_cluster_size->value();
  extractor.extractor.params.min_cluster_size = ui_->min_cluster_size->value();
  extractor.extractor.params.cluster_tolerance = ui_->cluster_tolerance->value();
  extractor.extractor.params.plane_distance_threshold = ui_->plane_distance_threshold->value();

  if (!client_->service_is_ready())
    throw std::runtime_error("Service is not available");

  auto request = std::make_shared<rviz_polygon_selection_tool::srv::GetSelection::Request>();
  auto future = client_->async_send_request(request);
  switch (future.wait_for(std::chrono::seconds(3)))
  {
    case std::future_status::ready:
      break;
    case std::future_status::timeout:
      throw std::runtime_error("Service call to '" + std::string(client_->get_service_name()) + "' timed out");
    default:
      throw std::runtime_error("Service call to '" + std::string(client_->get_service_name()) + "' failed");
  }

  rviz_polygon_selection_tool::srv::GetSelection::Response::SharedPtr response = future.get();

  return std::make_unique<ROISelectionMeshModifier>(node_, extractor, response->selection);
}

void ROISelectionMeshModifierWidget::configure(const YAML::Node& config)
{
  ui_->max_cluster_size->setValue(noether::getEntry<int>(config, MAX_CLUSTER_SIZE_KEY));
  ui_->min_cluster_size->setValue(noether::getEntry<int>(config, MIN_CLUSTER_SIZE_KEY));
  ui_->cluster_tolerance->setValue(noether::getEntry<double>(config, CLUSTER_TOLERANCE_KEY));
  ui_->plane_distance_threshold->setValue(noether::getEntry<double>(config, PLANE_DISTANCE_THRESHOLD_KEY));
}

void ROISelectionMeshModifierWidget::save(YAML::Node& config) const
{
  config[MAX_CLUSTER_SIZE_KEY] = ui_->max_cluster_size->value();
  config[MIN_CLUSTER_SIZE_KEY] = ui_->min_cluster_size->value();
  config[CLUSTER_TOLERANCE_KEY] = ui_->cluster_tolerance->value();
  config[PLANE_DISTANCE_THRESHOLD_KEY] = ui_->plane_distance_threshold->value();
}

}  // namespace snp_tpp
