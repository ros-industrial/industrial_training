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

#include <chrono>  // std::chono::seconds()
#include <memory>  // std::make_shared(), std::shared_ptr

#include <geometry_msgs/msg/pose.h>
#include <geometry_msgs/msg/pose_array.h>
#include <rclcpp/rclcpp.hpp>

#include <snp_msgs/msg/tool_path.h>
#include <snp_msgs/msg/tool_paths.h>
#include <snp_msgs/srv/generate_tool_paths.hpp>

namespace
{
geometry_msgs::msg::PoseArray compressPaths(const snp_msgs::msg::ToolPaths& tps)
{
  geometry_msgs::msg::PoseArray compressed;
  compressed.header.frame_id = "map";

  for (const snp_msgs::msg::ToolPath& tp : tps.paths)
  {
    for (const geometry_msgs::msg::PoseArray& pa : tp.segments)
    {
      for (const geometry_msgs::msg::Pose& p : pa.poses)
      {
        compressed.poses.push_back(p);
      }
    }
  }

  return compressed;
}
}  // namespace

int main(int argc, char** argv)
{
  // Initialize ROS2
  rclcpp::init(argc, argv);

  // Instantiate a ROS2 node & service client
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("tool_path_planning_client");
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseArray>> publisher =
      node->create_publisher<geometry_msgs::msg::PoseArray>("compressed_path", 2);
  rclcpp::Client<snp_msgs::srv::GenerateToolPaths>::SharedPtr client =
      node->create_client<snp_msgs::srv::GenerateToolPaths>("generate_tool_paths");
  if (!client->wait_for_service(std::chrono::seconds(10)))
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not find TPP server");
    return 1;
  }

  // Notify that this node is ready
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Tool Path Planning Client is ready");

  // Fill out the service call
  std::shared_ptr<snp_msgs::srv::GenerateToolPaths::Request> request =
      std::make_shared<snp_msgs::srv::GenerateToolPaths::Request>();
  request->mesh_filename = "/home/dmerz/Documents/sphere_cap/layer_4.ply";
  request->mesh_frame = "world";

  // Call the service
  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "TPP call failed");
    return 2;
  }

  // Compress to a single PoseArray
  geometry_msgs::msg::PoseArray compressed = compressPaths(result.get()->tool_paths);

  // Publish the compressed path
  publisher->publish(compressed);

  // Spin to accept service calls until ROS shuts down.
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
