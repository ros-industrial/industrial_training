#include "roi_selection_mesh_modifier.h"
#include "ui_roi_selection_mesh_modifier_widget.h"

#include <geometry_msgs/msg/point_stamped.hpp>
#include <noether_tpp/core/mesh_modifier.h>
#include <noether_filtering/submesh_extraction/extruded_polygon_mesh_extractor.h>
#include <noether_gui/plugin_interface.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/time.h>

namespace snp_tpp
{
ROISelectionMeshModifier::ROISelectionMeshModifier(rclcpp::Node::SharedPtr node,
                                                   noether::ExtrudedPolygonSubMeshExtractor extractor,
                                                   std::vector<geometry_msgs::msg::PointStamped> boundary)
  : boundary_(std::move(boundary)), extractor_(std::move(extractor)), buffer_(node->get_clock()), listener_(buffer_)
{
}

std::vector<pcl::PolygonMesh> ROISelectionMeshModifier::modify(const pcl::PolygonMesh& mesh) const
{
  if (boundary_.size() < 3)
  {
    return { mesh };
  }
  else
  {
    Eigen::MatrixX3d boundary(boundary_.size(), 3);
    for (Eigen::Index i = 0; i < boundary.rows(); ++i)
    {
      // Lookup transform between mesh header and
      Eigen::Isometry3d transform = tf2::transformToEigen(buffer_.lookupTransform(
          mesh.header.frame_id, boundary_[i].header.frame_id, tf2::TimePointZero, std::chrono::milliseconds(50)));

      Eigen::Vector3d v;
      tf2::fromMsg(boundary_[i].point, v);
      boundary.row(i) = transform * v;
    }

    return { extractor_.extract(mesh, boundary) };
  }
}

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
  extractor.params.max_cluster_size = ui_->max_cluster_size->value();
  extractor.params.min_cluster_size = ui_->min_cluster_size->value();
  extractor.params.cluster_tolerance = ui_->cluster_tolerance->value();
  extractor.params.plane_distance_threshold = ui_->plane_distance_threshold->value();

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

}  // namespace snp_tpp
