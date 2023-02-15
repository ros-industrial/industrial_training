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

#include <std_msgs/msg/header.hpp>
#include <yaml-cpp/yaml.h>
#include <builtin_interfaces/msg/time.hpp>

namespace YAML
{
template <>
struct convert<builtin_interfaces::msg::Time>
{
  static Node encode(const builtin_interfaces::msg::Time& rhs)
  {
    Node node;
    node["sec"] = rhs.sec;
    node["nanosec"] = rhs.nanosec;
    return node;
  }

  static bool decode(const Node& node, builtin_interfaces::msg::Time& rhs)
  {
    rhs.sec = node["sec"].as<uint32_t>();
    rhs.nanosec = node["nanosec"].as<uint32_t>();
    return true;
  }
};

template <>
struct convert<std_msgs::msg::Header>
{
  static Node encode(const std_msgs::msg::Header& rhs)
  {
    Node node;
    node["stamp"] = rhs.stamp;
    node["frame_id"] = rhs.frame_id;
    return node;
  }

  static bool decode(const Node& node, std_msgs::msg::Header& rhs)
  {
    rhs.stamp = node["stamp"].as<builtin_interfaces::msg::Time>();
    rhs.frame_id = node["frame_id"].as<std::string>();
    return true;
  }
};

// TODO: Duration

}  // namespace YAML
