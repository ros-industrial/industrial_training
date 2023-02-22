/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2016, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/core/demangle.hpp>
#include <Eigen/Geometry>
#include <tesseract_kinematics/ikfast/impl/ikfast_inv_kin.hpp>
#include <tesseract_kinematics/core/kinematics_plugin_factory.h>
#include "motoman_hc10_ikfast_solver.hpp"
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_kinematics
{
template <typename T>
T get(const YAML::Node& node, const std::string& key)
{
  try
  {
    return node[key].as<T>();
  }
  catch (const YAML::Exception&)
  {
    std::stringstream ss;
    ss << "Failed to extract '" << key << "' from configuration with type '" << boost::core::demangle(typeid(T).name());
    throw std::runtime_error(ss.str());
  }
}

class IKFastInvKinFactory : public InvKinFactory
{
public:
  virtual InverseKinematics::UPtr create(const std::string& solver_name,
                                         const tesseract_scene_graph::SceneGraph& scene_graph,
                                         const tesseract_scene_graph::SceneState& /*scene_state*/,
                                         const KinematicsPluginFactory& /*plugin_factory*/,
                                         const YAML::Node& config) const
  {
    auto base_link = get<std::string>(config, "base_link");
    auto tip_link = get<std::string>(config, "tip_link");
    auto n_joints = get<std::size_t>(config, "n_joints");

    // Get the free joint states
    std::map<std::size_t, std::vector<double>> free_joint_states_map;
    try
    {
      const YAML::Node& free_joint_states_node = config["free_joint_states"];
      for (auto it = free_joint_states_node.begin(); it != free_joint_states_node.end(); ++it)
      {
        free_joint_states_map[it->first.as<std::size_t>()] = it->second.as<std::vector<double>>();
      }
    }
    catch (const std::exception&)
    {
      CONSOLE_BRIDGE_logDebug("No free joint states specified for IKFast plugin");
    }

    // Get the active joints in between the base link and tip link
    tesseract_scene_graph::ShortestPath path = scene_graph.getShortestPath(base_link, tip_link);

    // Check the joints specification
    if (path.active_joints.size() != n_joints + free_joint_states_map.size())
    {
      std::stringstream ss;
      ss << "Number of active joints (" << path.active_joints.size()
         << ") must equal the sum of the number of nominal joints (" << n_joints << ") and the number of free joints ("
         << free_joint_states_map.size() << ")";
      throw std::runtime_error(ss.str());
    }

    // Get the redundancy capable joint indices
    std::vector<Eigen::Index> redundancy_capable_joint_indices;
    try
    {
      redundancy_capable_joint_indices = config["redundancy_capable_joint_indices"].as<std::vector<Eigen::Index>>();
    }
    catch (const std::exception&)
    {
      CONSOLE_BRIDGE_logDebug("Using default redundancy capable joints for IKFast plugin");

      for (std::size_t i = 0; i < path.active_joints.size(); ++i)
      {
        if (free_joint_states_map.find(i) == free_joint_states_map.end())
        {
          const auto& joint = scene_graph.getJoint(path.active_joints.at(i));
          if (joint->limits->upper - joint->limits->lower > 2 * M_PI)
          {
            redundancy_capable_joint_indices.push_back(static_cast<Eigen::Index>(i));
          }
        }
      }
    }

    std::vector<std::vector<double>> free_joint_states;
    free_joint_states.reserve(free_joint_states_map.size());
    std::transform(free_joint_states_map.begin(), free_joint_states_map.end(), std::back_inserter(free_joint_states),
                   [](const std::pair<const std::size_t, std::vector<double>>& pair) { return pair.second; });

    return std::make_unique<IKFastInvKin>(base_link, tip_link, path.active_joints, redundancy_capable_joint_indices,
                                          solver_name, free_joint_states);
  }
};

}  // namespace tesseract_kinematics

TESSERACT_ADD_PLUGIN(tesseract_kinematics::IKFastInvKinFactory, MotomanHC10InvKinFactory)
