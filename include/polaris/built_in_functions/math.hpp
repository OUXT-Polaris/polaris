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

#ifndef POLARIS__BUILT_IN_FUNCTIONS__MATH_HPP_
#define POLARIS__BUILT_IN_FUNCTIONS__MATH_HPP_

#include <peglib.h>

#include <polaris/types/type_base.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <boost/optional.hpp>
#include <boost/any.hpp>

#include <functional>
#include <unordered_map>
#include <memory>

namespace polaris
{
namespace built_in_functions
{
namespace math
{
boost::any construct_double(std::shared_ptr<peg::Ast> ast)
{
  if (ast->name == "DOUBLE") {
    try {
      types::TypeBase<double> double_value;
      double_value.setValue(std::stod(ast->token));
      return double_value;
    } catch (std::invalid_argument) {
      throw std::runtime_error("failed to parse token into double value, std::invalid_argument");
    } catch (std::out_of_range) {
      throw std::runtime_error("failed to parse token into double value, std::out_of_range");
    }
  }
  return boost::none;
}

boost::any construct_quaternion(std::shared_ptr<peg::Ast> ast)
{
  geometry_msgs::msg::Quaternion quat;
  if (ast->name == "ARGUMENTS") {
    try {
      quat.x = boost::any_cast<types::TypeBase<double>>(construct_double(ast->nodes[0])).getValue();
      quat.y = boost::any_cast<types::TypeBase<double>>(construct_double(ast->nodes[1])).getValue();
      quat.z = boost::any_cast<types::TypeBase<double>>(construct_double(ast->nodes[2])).getValue();
      quat.w = boost::any_cast<types::TypeBase<double>>(construct_double(ast->nodes[3])).getValue();
      types::TypeBase<geometry_msgs::msg::Quaternion> quat_value;
      quat_value.setValue(quat);
      return quat_value;
    } catch (boost::bad_any_cast) {
      std::runtime_error(
        "failed to cast as double value in constructing quaternion, boost::bad_any_cast");
    }
  }
  return boost::none;
}
}  // namespace math
}  // namespace built_in_functions
}  // namespace polaris
#endif  // POLARIS__BUILT_IN_FUNCTIONS__MATH_HPP_
