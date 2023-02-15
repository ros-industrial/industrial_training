/**
 * @copyright Copyright (c) 2021, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <memory>      // std::make_shared(), std::shared_ptr
#include <string>      // std::string
#include <functional>  // std::bind(), std::placeholders

#include <snp_msgs/srv/generate_tool_paths.hpp>

#include <noether_tpp/core/types.h>
#include <noether_tpp/tool_path_planners/raster/direction_generators.h>
#include <noether_tpp/tool_path_planners/raster/origin_generators.h>
#include <noether_tpp/tool_path_planners/raster/plane_slicer_raster_planner.h>
#include <noether_tpp/tool_path_modifiers/organization_modifiers.h>
#include <pcl/conversions.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/Vertices.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.h>

namespace
{
template <typename T>
T declareAndGet(rclcpp::Node* node, const std::string& key)
{
  T val;
  node->declare_parameter(key);
  if (!node->get_parameter(key, val))
  {
    throw std::runtime_error("Failed to get '" + key + "' parameter");
  }
  return val;
}

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
class TPPNode : public rclcpp::Node
{
public:
  TPPNode(const std::string& name)
    : rclcpp::Node(name)
    , line_spacing_(declareAndGet<double>(this, "line_spacing"))
    , point_spacing_(declareAndGet<double>(this, "point_spacing"))
    , min_segment_size_(declareAndGet<double>(this, "min_segment_size"))
    , min_hole_size_(declareAndGet<double>(this, "min_hole_size"))
    , search_radius_(declareAndGet<double>(this, "search_radius"))
  {
    srvr_ = this->create_service<snp_msgs::srv::GenerateToolPaths>(
        "generate_tool_paths", std::bind(&TPPNode::callPlanner, this, std::placeholders::_1, std::placeholders::_2));
    return;
  }

  void callPlanner(const std::shared_ptr<snp_msgs::srv::GenerateToolPaths::Request> request,
                   const std::shared_ptr<snp_msgs::srv::GenerateToolPaths::Response> response)
  {
    response->success = true;

    // Convert shape_msgs::Mesh to pcl::PolygonMesh
    pcl::PolygonMesh pcl_mesh;
    if (pcl::io::loadPolygonFile(request->mesh_filename, pcl_mesh) == 0)
    {
      response->success = false;
      response->message = "Could not open mesh file `" + request->mesh_filename + "`";
    }
    else
    {
      // Create a planner
      noether::PlaneSlicerRasterPlanner planner(std::make_unique<noether::PrincipalAxisDirectionGenerator>(),
                                                std::make_unique<noether::FixedOriginGenerator>());

      // Configure the planner
      planner.setLineSpacing(line_spacing_);
      planner.setMinHoleSize(min_hole_size_);
      planner.setMinSegmentSize(min_segment_size_);
      planner.setPointSpacing(point_spacing_);
      planner.setSearchRadius(search_radius_);

      // Create a modifier to organize the tool path in a snake pattern
      noether::SnakeOrganizationModifier mod;

      // Call the planner
      noether::ToolPaths paths = mod.modify(planner.plan(pcl_mesh));

      // Convert noether::ToolPaths to snp_msgs::msg::ToolPaths
      response->tool_paths = toMsg(paths);

      if (paths.empty())
      {
        response->success = false;
        response->message = "Path generation failed";
      }
    }

    return;
  }

private:
  rclcpp::Service<snp_msgs::srv::GenerateToolPaths>::SharedPtr srvr_;
  const double line_spacing_;
  const double point_spacing_;
  const double min_segment_size_;
  const double min_hole_size_;
  const double search_radius_;
};  // class TPPNode

}  // namespace snp_tpp

int main(int argc, char** argv)
{
  // Initialize ROS2
  rclcpp::init(argc, argv);

  // Instantiate a ROS2 node
  std::shared_ptr<rclcpp::Node> tpp_node = std::make_shared<snp_tpp::TPPNode>("tool_path_planning_server");

  // Notify that this node is ready
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Tool Path Planning Server is ready");

  // Spin to accept service calls until ROS shuts down.
  rclcpp::spin(tpp_node);
  rclcpp::shutdown();
  return 0;
}
