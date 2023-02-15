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

#include <fstream>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>

namespace message_serialization
{
/**
 * @brief Serializes an input object to a YAML-formatted file
 * @param val
 * @param file
 * @throws exception on failure to open or write to a file stream
 */
template <class T>
inline void serialize(const T& val, const std::string& file)
{
  std::ofstream ofh(file);
  if (!ofh)
    throw std::runtime_error("Failed to open output file stream at '" + file + "'");

  YAML::Node n = YAML::Node(val);
  ofh << n;
}

/**
 * @brief Deserializes a YAML-formatted file into a specific object type
 * @param file
 * @return
 * @throws exception when unable to load the file or convert it to the specified type
 */
template <class T>
inline T deserialize(const std::string& file)
{
  YAML::Node node;
  node = YAML::LoadFile(file);
  return node.as<T>();
}

}  // namespace message_serialization
