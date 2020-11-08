// Copyright (c) 2020, OUXT-Polaris
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef POLARIS__TYPES__ENTITY_HPP_
#define POLARIS__TYPES__ENTITY_HPP_

#include <geometry_msgs/msg/pose.hpp>

#include <string>
#include <vector>

namespace polaris
{
namespace types
{
class Entity
{
public:
  Entity(
    geometry_msgs::msg::Pose pose, std::vector<std::string> type,
    std::vector<geometry_msgs::msg::Point> polygon)
  : pose(pose), type(type), polygon(polygon)
  {}
  Entity(
    geometry_msgs::msg::Pose pose, std::string type,
    std::vector<geometry_msgs::msg::Point> polygon)
  : pose(pose), type({type}), polygon(polygon)
  {}
  const geometry_msgs::msg::Pose pose;
  const std::vector<std::string> type;
  const std::vector<geometry_msgs::msg::Point> polygon;
};
}  // namespace types
}  // namespace polaris
#endif  // POLARIS__TYPES__ENTITY_HPP_
