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

#include <polaris/types/type_base.hpp>
#include <polaris/built_in_functions/functions.hpp>

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
boost::any Functions::constructInteger(std::shared_ptr<peg::Ast> ast)
{
  if (ast->name == "INTEGER") {
    try {
      types::TypeBase<int> int_value;
      int_value.setValue(std::stoi(ast->token));
      return int_value;
    } catch (std::invalid_argument) {
      throw std::runtime_error("failed to parse token into int value, std::invalid_argument");
    } catch (std::out_of_range) {
      throw std::runtime_error("failed to parse token into int value, std::out_of_range");
    }
  }
  return boost::none;
}

boost::any Functions::constructDouble(std::shared_ptr<peg::Ast> ast)
{
  if (ast->name == "DOUBLE" || ast->name == "INTEGER") {
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

boost::any Functions::constructDuaternion(std::shared_ptr<peg::Ast> ast)
{
  geometry_msgs::msg::Quaternion quat;
  if (ast->name == "ARGUMENTS") {
    try {
      if (ast->nodes[0]->name == "CALL") {
        auto val = evaluate(ast->nodes[0]->nodes[0]->token, ast->nodes[0]->nodes[1]);
        if (val.type() != typeid(types::TypeBase<double>)) {
          throw std::runtime_error("failed to interprit as double value");
        }
        quat.x = boost::any_cast<types::TypeBase<double>>(val).getValue();
      } else {
        quat.x =
          boost::any_cast<types::TypeBase<double>>(constructDouble(ast->nodes[0])).getValue();
      }
      if (ast->nodes[1]->name == "CALL") {
        auto val = evaluate(ast->nodes[1]->nodes[0]->token, ast->nodes[1]->nodes[1]);
        if (val.type() != typeid(types::TypeBase<double>)) {
          throw std::runtime_error("failed to interprit as double value");
        }
        quat.y = boost::any_cast<types::TypeBase<double>>(val).getValue();
      } else {
        quat.y =
          boost::any_cast<types::TypeBase<double>>(constructDouble(ast->nodes[1])).getValue();
      }
      if (ast->nodes[2]->name == "CALL") {
        auto val = evaluate(ast->nodes[2]->nodes[0]->token, ast->nodes[2]->nodes[1]);
        if (val.type() != typeid(types::TypeBase<double>)) {
          throw std::runtime_error("failed to interprit as double value");
        }
        quat.z = boost::any_cast<types::TypeBase<double>>(val).getValue();
      } else {
        quat.z =
          boost::any_cast<types::TypeBase<double>>(constructDouble(ast->nodes[2])).getValue();
      }
      if (ast->nodes[2]->name == "CALL") {
        auto val = evaluate(ast->nodes[3]->nodes[0]->token, ast->nodes[3]->nodes[1]);
        if (val.type() != typeid(types::TypeBase<double>)) {
          throw std::runtime_error("failed to interprit as double value");
        }
        quat.w = boost::any_cast<types::TypeBase<double>>(val).getValue();
      } else {
        quat.w =
          boost::any_cast<types::TypeBase<double>>(constructDouble(ast->nodes[3])).getValue();
      }
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
}  // namespace built_in_functions
}  // namespace polaris
