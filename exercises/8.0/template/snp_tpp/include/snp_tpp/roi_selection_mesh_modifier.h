#pragma once

#include <geometry_msgs/msg/point_stamped.hpp>
#include <noether_tpp/core/mesh_modifier.h>
#include <noether_filtering/subset_extraction/extruded_polygon_subset_extractor.h>
#include <rclcpp/node.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

namespace snp_tpp
{
class ROISelectionMeshModifier : public noether::MeshModifier
{
public:
  ROISelectionMeshModifier(rclcpp::Node::SharedPtr node, noether::ExtrudedPolygonSubMeshExtractor extractor,
                           std::vector<geometry_msgs::msg::PointStamped> boundary);

  std::vector<pcl::PolygonMesh> modify(const pcl::PolygonMesh& mesh) const override;

private:
  const std::vector<geometry_msgs::msg::PointStamped> boundary_;
  const noether::ExtrudedPolygonSubMeshExtractor extractor_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
};

}  // namespace snp_tpp
