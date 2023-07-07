#include <snp_tpp/roi_selection_mesh_modifier.h>
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

}  // namespace snp_tpp
