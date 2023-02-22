/*
 * Copyright 2018 Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include "std_msgs_yaml.h"

namespace YAML
{
template <>
struct convert<trajectory_msgs::msg::JointTrajectoryPoint>
{
  static Node encode(const trajectory_msgs::msg::JointTrajectoryPoint& rhs)
  {
    Node node;
    node["positions"] = rhs.positions;
    node["velocities"] = rhs.velocities;
    node["accelerations"] = rhs.accelerations;
    node["effort"] = rhs.effort;
    node["time_from_start"]["sec"] = rhs.time_from_start.sec;
    node["time_from_start"]["nanosec"] = rhs.time_from_start.nanosec;
    return node;
  }

  static bool decode(const Node& node, trajectory_msgs::msg::JointTrajectoryPoint& rhs)
  {
    rhs.positions = node["positions"].as<std::vector<double>>();
    rhs.velocities = node["velocities"].as<std::vector<double>>();
    rhs.accelerations = node["accelerations"].as<std::vector<double>>();
    rhs.effort = node["effort"].as<std::vector<double>>();
    rhs.time_from_start.sec = node["time_from_start"]["sec"].as<uint32_t>();
    rhs.time_from_start.nanosec = node["time_from_start"]["nanosec"].as<uint32_t>();
    return true;
  }
};

template <>
struct convert<trajectory_msgs::msg::JointTrajectory>
{
  static Node encode(const trajectory_msgs::msg::JointTrajectory& rhs)
  {
    Node node;
    node["header"] = rhs.header;
    node["joint_names"] = rhs.joint_names;
    node["points"] = rhs.points;
    return node;
  }

  static bool decode(const Node& node, trajectory_msgs::msg::JointTrajectory& rhs)
  {
    rhs.header = node["header"].as<std_msgs::msg::Header>();
    rhs.joint_names = node["joint_names"].as<std::vector<std::string>>();
    rhs.points = node["points"].as<std::vector<trajectory_msgs::msg::JointTrajectoryPoint>>();
    return true;
  }
};

}  // namespace YAML
