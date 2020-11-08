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
#include <polaris/exception.hpp>

#include <quaternion_operation/quaternion_operation.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <boost/optional.hpp>
#include <boost/any.hpp>

#include <functional>
#include <unordered_map>
#include <memory>
#include <string>
#include <vector>

namespace polaris
{
namespace built_in_functions
{
boost::any Functions::fetchVariable(std::shared_ptr<peg::Ast> ast)
{
  return variables_[ast->token];
}

boost::any Functions::constructEntity(std::shared_ptr<peg::Ast> ast)
{

  auto pose_value = evaluate(ast->nodes[0]->name, ast->nodes[0]);
  geometry_msgs::msg::Pose pose;
  if (pose_value.type() == typeid(types::TypeBase<geometry_msgs::msg::Pose>)) {
    pose = boost::any_cast<types::TypeBase<geometry_msgs::msg::Pose>>(pose_value).getValue();
  } else {
    POLARIS_THROW_EVALUATION_ERROR(ast->nodes[0], "failed to parse pose");
  }
  auto type_value = evaluate(ast->nodes[1]->name, ast->nodes[1]);
  std::string type;
  std::vector<std::string> types;
  if (type_value.type() == typeid(types::TypeBase<std::string>)) {
    type = boost::any_cast<types::TypeBase<std::string>>(type_value).getValue();
  } else if (type_value.type() == typeid(types::TypeBase<std::vector<std::string>>)) {
    types = boost::any_cast<types::TypeBase<std::vector<std::string>>>(type_value).getValue();
  } else {
    POLARIS_THROW_EVALUATION_ERROR(ast->nodes[1], "failed to parse types");
  }
  auto polygon_value = evaluate(ast->nodes[2]->name, ast->nodes[2]);
  std::vector<geometry_msgs::msg::Point> polygon;
  if (polygon_value.type() == typeid(types::TypeBase<std::vector<geometry_msgs::msg::Point>>)) {
    polygon =
      boost::any_cast<types::TypeBase<std::vector<geometry_msgs::msg::Point>>>(polygon_value).
      getValue();
  } else {
    POLARIS_THROW_EVALUATION_ERROR(ast->nodes[2], "failed to parse polygon");
  }
  if (types.size() == 0) {
    return types::TypeBase<polaris::types::Entity>(polaris::types::Entity(pose, type, polygon));
  } else {
    return types::TypeBase<polaris::types::Entity>(polaris::types::Entity(pose, types, polygon));
  }
  return boost::none;
}

boost::any Functions::constructArray(std::shared_ptr<peg::Ast> ast)
{
  if (ast->nodes.size() == 0) {
    POLARIS_THROW_EVALUATION_ERROR(ast,
      "array is empty");
  }
  const auto first_value = evaluate(ast->nodes[0]->name, ast->nodes[0]);
  const auto & value_type = first_value.type();
  if (value_type == typeid(types::TypeBase<int>)) {
    types::TypeBase<std::vector<int>> array;
    std::vector<int> array_value;
    for (const auto & node : ast->nodes) {
      auto value = evaluate(node->name, node);
      if (value.type() == typeid(types::TypeBase<int>)) {
        array_value.emplace_back(boost::any_cast<types::TypeBase<int>>(value).getValue());
      } else {
        POLARIS_THROW_EVALUATION_ERROR(ast, "array value is not int");
      }
    }
    array.setValue(array_value);
    return array;
  }
  if (value_type == typeid(types::TypeBase<double>)) {
    types::TypeBase<std::vector<double>> array;
    std::vector<double> array_value;
    for (const auto & node : ast->nodes) {
      auto value = evaluate(node->name, node);
      if (value.type() == typeid(types::TypeBase<double>)) {
        array_value.emplace_back(boost::any_cast<types::TypeBase<double>>(value).getValue());
      } else {
        POLARIS_THROW_EVALUATION_ERROR(ast, "array value is not double");
      }
    }
    array.setValue(array_value);
    return array;
  }
  if (value_type == typeid(types::TypeBase<std::string>)) {
    types::TypeBase<std::vector<std::string>> array;
    std::vector<std::string> array_value;
    for (const auto & node : ast->nodes) {
      auto value = evaluate(node->name, node);
      if (value.type() == typeid(types::TypeBase<std::string>)) {
        array_value.emplace_back(boost::any_cast<types::TypeBase<std::string>>(value).getValue());
      } else {
        POLARIS_THROW_EVALUATION_ERROR(ast, "array value is not string");
      }
    }
    array.setValue(array_value);
    return array;
  }
  if (value_type == typeid(types::TypeBase<geometry_msgs::msg::Quaternion>)) {
    types::TypeBase<std::vector<geometry_msgs::msg::Quaternion>> array;
    std::vector<geometry_msgs::msg::Quaternion> array_value;
    for (const auto & node : ast->nodes) {
      auto value = evaluate(node->name, node);
      if (value.type() == typeid(types::TypeBase<geometry_msgs::msg::Quaternion>)) {
        array_value.emplace_back(boost::any_cast<types::TypeBase<geometry_msgs::msg::Quaternion>>(
            value).getValue());
      } else {
        POLARIS_THROW_EVALUATION_ERROR(ast, "array value is not quaternion");
      }
    }
    array.setValue(array_value);
    return array;
  }
  if (value_type == typeid(types::TypeBase<geometry_msgs::msg::Point>)) {
    types::TypeBase<std::vector<geometry_msgs::msg::Point>> array;
    std::vector<geometry_msgs::msg::Point> array_value;
    for (const auto & node : ast->nodes) {
      auto value = evaluate(node->name, node);
      if (value.type() == typeid(types::TypeBase<geometry_msgs::msg::Point>)) {
        array_value.emplace_back(boost::any_cast<types::TypeBase<geometry_msgs::msg::Point>>(
            value).getValue());
      } else {
        POLARIS_THROW_EVALUATION_ERROR(ast, "array value is not point");
      }
    }
    array.setValue(array_value);
    return array;
  }
  if (value_type == typeid(types::TypeBase<geometry_msgs::msg::Pose>)) {
    types::TypeBase<std::vector<geometry_msgs::msg::Pose>> array;
    std::vector<geometry_msgs::msg::Pose> array_value;
    for (const auto & node : ast->nodes) {
      auto value = evaluate(node->name, node);
      if (value.type() == typeid(types::TypeBase<geometry_msgs::msg::Pose>)) {
        array_value.emplace_back(boost::any_cast<types::TypeBase<geometry_msgs::msg::Pose>>(
            value).getValue());
      } else {
        POLARIS_THROW_EVALUATION_ERROR(ast, "array value is not pose");
      }
    }
    array.setValue(array_value);
    return array;
  }
  return boost::none;
}

boost::any Functions::constructString(std::shared_ptr<peg::Ast> ast)
{
  if (ast->name == "STRING") {
    types::TypeBase<std::string> str_value;
    str_value.setValue(ast->token);
    return str_value;
  }
  return boost::none;
}

boost::any Functions::constructInteger(std::shared_ptr<peg::Ast> ast)
{
  if (ast->name == "INTEGER") {
    try {
      types::TypeBase<int> int_value;
      int_value.setValue(std::stoi(ast->token));
      return int_value;
    } catch (std::invalid_argument) {
      POLARIS_THROW_EVALUATION_ERROR(ast,
        "failed to parse token into int value, std::invalid_argument");
    } catch (std::out_of_range) {
      POLARIS_THROW_EVALUATION_ERROR(ast,
        "failed to parse token into int value, std::out_of_range");
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
      POLARIS_THROW_EVALUATION_ERROR(ast,
        "failed to parse token into double value, std::invalid_argument");
    } catch (std::out_of_range) {
      POLARIS_THROW_EVALUATION_ERROR(ast,
        "failed to parse token into double value, std::out_of_range");
    }
  }
  return boost::none;
}

boost::any Functions::constructPose(std::shared_ptr<peg::Ast> ast)
{
  geometry_msgs::msg::Pose pose;
  if (ast->name == "ARGUMENTS") {
    if (ast->nodes[0]->name == "CALL") {
      if (ast->nodes[0]->nodes[0]->name == "IDENTIFIER") {
        boost::any val = evaluate(ast->nodes[0]->nodes[0]->token, ast->nodes[0]);
        if (ast->nodes[0]->nodes[0]->token == "point") {
          if (val.type() == typeid(types::TypeBase<geometry_msgs::msg::Point>)) {
            auto p = boost::any_cast<types::TypeBase<geometry_msgs::msg::Point>>(val).getValue();
            pose.position = p;
          } else {
            POLARIS_THROW_EVALUATION_ERROR(ast,
              "constracting point type failed");
          }
        } else {
          POLARIS_THROW_EVALUATION_ERROR(ast,
            "first argument shold be point type");
        }
      }
    } else if (ast->nodes[0]->name == "IDENTIFIER") {
      if (variables_.count(ast->nodes[0]->token) == 0) {
        POLARIS_THROW_EVALUATION_ERROR(ast,
          "variable " + ast->nodes[0]->token + " did not difined.");
      }
      if (variables_[ast->nodes[0]->token].type() ==
        typeid(types::TypeBase<geometry_msgs::msg::Point>))
      {
        pose.position =
          boost::any_cast<types::TypeBase<geometry_msgs::msg::Point>>(variables_[ast->nodes[0]->
            token]).getValue();
      }
    } else {
      POLARIS_THROW_EVALUATION_ERROR(ast, "name of the node is invalid");
    }
    if (ast->nodes[1]->name == "CALL") {
      if (ast->nodes[1]->nodes[0]->name == "IDENTIFIER") {
        boost::any val = evaluate(ast->nodes[1]->nodes[0]->token, ast->nodes[1]);
        if (ast->nodes[1]->nodes[0]->token == "quaternion") {
          if (val.type() == typeid(types::TypeBase<geometry_msgs::msg::Quaternion>)) {
            auto q =
              boost::any_cast<types::TypeBase<geometry_msgs::msg::Quaternion>>(val).getValue();
            pose.orientation = q;
          } else {
            POLARIS_THROW_EVALUATION_ERROR(ast,
              "constracting quaternion type failed");
          }
        } else {
          POLARIS_THROW_EVALUATION_ERROR(ast,
            "second argument shold be quaternion type");
        }
      }
    } else if (ast->nodes[1]->name == "IDENTIFIER") {
      if (variables_.count(ast->nodes[1]->token) == 0) {
        POLARIS_THROW_EVALUATION_ERROR(ast,
          "variable " + ast->nodes[1]->token + " did not difined.");
      }
      if (variables_[ast->nodes[1]->token].type() ==
        typeid(types::TypeBase<geometry_msgs::msg::Quaternion>))
      {
        pose.orientation =
          boost::any_cast<types::TypeBase<geometry_msgs::msg::Quaternion>>(
          variables_[ast->nodes[1]->token]).getValue();
      }
    } else {
      POLARIS_THROW_EVALUATION_ERROR(ast, "name of the node is invalid");
    }
  }
  types::TypeBase<geometry_msgs::msg::Pose> pose_value;
  pose_value.setValue(pose);
  return pose_value;
}

boost::any Functions::constructPoint(std::shared_ptr<peg::Ast> ast)
{
  geometry_msgs::msg::Point point;
  if (ast->name == "ARGUMENTS") {
    try {
      if (ast->nodes[0]->name == "CALL") {
        auto val = evaluate(ast->nodes[0]->nodes[0]->token, ast->nodes[0]->nodes[1]);
        if (val.type() != typeid(types::TypeBase<double>)) {
          POLARIS_THROW_EVALUATION_ERROR(ast, "failed to interprit as double value");
        }
        point.x = boost::any_cast<types::TypeBase<double>>(val).getValue();
      } else if (ast->nodes[0]->name == "IDENTIFIER") {
        point.x =
          boost::any_cast<types::TypeBase<double>>(variables_[ast->nodes[0]->token]).getValue();
      } else {
        point.x =
          boost::any_cast<types::TypeBase<double>>(constructDouble(ast->nodes[0])).getValue();
      }
      if (ast->nodes[1]->name == "CALL") {
        auto val = evaluate(ast->nodes[1]->nodes[0]->token, ast->nodes[1]->nodes[1]);
        if (val.type() != typeid(types::TypeBase<double>)) {
          POLARIS_THROW_EVALUATION_ERROR(ast, "failed to interprit as double value");
        }
        point.y = boost::any_cast<types::TypeBase<double>>(val).getValue();
      } else if (ast->nodes[1]->name == "IDENTIFIER") {
        point.y =
          boost::any_cast<types::TypeBase<double>>(variables_[ast->nodes[1]->token]).getValue();
      } else {
        point.y =
          boost::any_cast<types::TypeBase<double>>(constructDouble(ast->nodes[1])).getValue();
      }
      if (ast->nodes[2]->name == "CALL") {
        auto val = evaluate(ast->nodes[2]->nodes[0]->token, ast->nodes[2]->nodes[1]);
        if (val.type() != typeid(types::TypeBase<double>)) {
          POLARIS_THROW_EVALUATION_ERROR(ast, "failed to interprit as double value");
        }
        point.z = boost::any_cast<types::TypeBase<double>>(val).getValue();
      } else if (ast->nodes[2]->name == "IDENTIFIER") {
        point.z =
          boost::any_cast<types::TypeBase<double>>(variables_[ast->nodes[2]->token]).getValue();
      } else {
        point.z =
          boost::any_cast<types::TypeBase<double>>(constructDouble(ast->nodes[2])).getValue();
      }
      types::TypeBase<geometry_msgs::msg::Point> point_value;
      point_value.setValue(point);
      return point_value;
    } catch (boost::bad_any_cast) {
      POLARIS_THROW_EVALUATION_ERROR(ast,
        "failed to cast as double value in constructing point, boost::bad_any_cast");
    }
  }
  return boost::none;
}

boost::any Functions::constructQuaternionFromRpy(std::shared_ptr<peg::Ast> ast)
{
  geometry_msgs::msg::Vector3 rpy;
  if (ast->name == "ARGUMENTS") {
    try {
      if (ast->nodes[0]->name == "CALL") {
        auto val = evaluate(ast->nodes[0]->nodes[0]->token, ast->nodes[0]->nodes[1]);
        if (val.type() != typeid(types::TypeBase<double>)) {
          POLARIS_THROW_EVALUATION_ERROR(ast, "failed to interprit as double value");
        }
        rpy.x = boost::any_cast<types::TypeBase<double>>(val).getValue();
      } else if (ast->nodes[0]->name == "IDENTIFIER") {
        rpy.x =
          boost::any_cast<types::TypeBase<double>>(variables_[ast->nodes[0]->token]).getValue();
      } else {
        rpy.x =
          boost::any_cast<types::TypeBase<double>>(constructDouble(ast->nodes[0])).getValue();
      }
      if (ast->nodes[1]->name == "CALL") {
        auto val = evaluate(ast->nodes[1]->nodes[0]->token, ast->nodes[1]->nodes[1]);
        if (val.type() != typeid(types::TypeBase<double>)) {
          POLARIS_THROW_EVALUATION_ERROR(ast, "failed to interprit as double value");
        }
        rpy.y = boost::any_cast<types::TypeBase<double>>(val).getValue();
      } else if (ast->nodes[1]->name == "IDENTIFIER") {
        rpy.y =
          boost::any_cast<types::TypeBase<double>>(variables_[ast->nodes[1]->token]).getValue();
      } else {
        rpy.y =
          boost::any_cast<types::TypeBase<double>>(constructDouble(ast->nodes[1])).getValue();
      }
      if (ast->nodes[2]->name == "CALL") {
        auto val = evaluate(ast->nodes[2]->nodes[0]->token, ast->nodes[2]->nodes[1]);
        if (val.type() != typeid(types::TypeBase<double>)) {
          POLARIS_THROW_EVALUATION_ERROR(ast, "failed to interprit as double value");
        }
        rpy.z = boost::any_cast<types::TypeBase<double>>(val).getValue();
      } else if (ast->nodes[2]->name == "IDENTIFIER") {
        rpy.z =
          boost::any_cast<types::TypeBase<double>>(variables_[ast->nodes[2]->token]).getValue();
      } else {
        rpy.z =
          boost::any_cast<types::TypeBase<double>>(constructDouble(ast->nodes[2])).getValue();
      }
      types::TypeBase<geometry_msgs::msg::Quaternion> quat_value;
      geometry_msgs::msg::Quaternion quat =
        quaternion_operation::convertEulerAngleToQuaternion(rpy);
      quat_value.setValue(quat);
      return quat_value;
    } catch (boost::bad_any_cast) {
      POLARIS_THROW_EVALUATION_ERROR(ast,
        "failed to cast as double value in constructing quaternion, boost::bad_any_cast");
    }
  }
  return boost::none;
}

boost::any Functions::constructQuaternion(std::shared_ptr<peg::Ast> ast)
{
  geometry_msgs::msg::Quaternion quat;
  if (ast->name == "ARGUMENTS") {
    try {
      if (ast->nodes[0]->name == "CALL") {
        auto val = evaluate(ast->nodes[0]->nodes[0]->token, ast->nodes[0]->nodes[1]);
        if (val.type() != typeid(types::TypeBase<double>)) {
          POLARIS_THROW_EVALUATION_ERROR(ast, "failed to interprit as double value");
        }
        quat.x = boost::any_cast<types::TypeBase<double>>(val).getValue();
      } else if (ast->nodes[0]->name == "IDENTIFIER") {
        quat.x =
          boost::any_cast<types::TypeBase<double>>(variables_[ast->nodes[0]->token]).getValue();
      } else {
        quat.x =
          boost::any_cast<types::TypeBase<double>>(constructDouble(ast->nodes[0])).getValue();
      }
      if (ast->nodes[1]->name == "CALL") {
        auto val = evaluate(ast->nodes[1]->nodes[0]->token, ast->nodes[1]->nodes[1]);
        if (val.type() != typeid(types::TypeBase<double>)) {
          POLARIS_THROW_EVALUATION_ERROR(ast, "failed to interprit as double value");
        }
        quat.y = boost::any_cast<types::TypeBase<double>>(val).getValue();
      } else if (ast->nodes[1]->name == "IDENTIFIER") {
        quat.y =
          boost::any_cast<types::TypeBase<double>>(variables_[ast->nodes[1]->token]).getValue();
      } else {
        quat.y =
          boost::any_cast<types::TypeBase<double>>(constructDouble(ast->nodes[1])).getValue();
      }
      if (ast->nodes[2]->name == "CALL") {
        auto val = evaluate(ast->nodes[2]->nodes[0]->token, ast->nodes[2]->nodes[1]);
        if (val.type() != typeid(types::TypeBase<double>)) {
          POLARIS_THROW_EVALUATION_ERROR(ast, "failed to interprit as double value");
        }
        quat.z = boost::any_cast<types::TypeBase<double>>(val).getValue();
      } else if (ast->nodes[2]->name == "IDENTIFIER") {
        quat.z =
          boost::any_cast<types::TypeBase<double>>(variables_[ast->nodes[2]->token]).getValue();
      } else {
        quat.z =
          boost::any_cast<types::TypeBase<double>>(constructDouble(ast->nodes[2])).getValue();
      }
      if (ast->nodes[3]->name == "CALL") {
        auto val = evaluate(ast->nodes[3]->nodes[0]->token, ast->nodes[3]->nodes[1]);
        if (val.type() != typeid(types::TypeBase<double>)) {
          POLARIS_THROW_EVALUATION_ERROR(ast, "failed to interprit as double value");
        }
        quat.w = boost::any_cast<types::TypeBase<double>>(val).getValue();
      } else if (ast->nodes[3]->name == "IDENTIFIER") {
        quat.w =
          boost::any_cast<types::TypeBase<double>>(variables_[ast->nodes[3]->token]).getValue();
      } else {
        quat.w =
          boost::any_cast<types::TypeBase<double>>(constructDouble(ast->nodes[3])).getValue();
      }
      types::TypeBase<geometry_msgs::msg::Quaternion> quat_value;
      quat_value.setValue(quat);
      return quat_value;
    } catch (boost::bad_any_cast) {
      POLARIS_THROW_EVALUATION_ERROR(ast,
        "failed to cast as double value in constructing quaternion, boost::bad_any_cast");
    }
  }
  return boost::none;
}

boost::any Functions::multiplication(std::shared_ptr<peg::Ast> ast)
{
  auto v0 = evaluate(ast->nodes[0]->name, ast->nodes[0]);
  auto v1 = evaluate(ast->nodes[2]->name, ast->nodes[2]);
  // (quaternion value) * (quaternion value)
  if (v0.type() == typeid(types::TypeBase<geometry_msgs::msg::Quaternion>) &&
    v1.type() == typeid(types::TypeBase<geometry_msgs::msg::Quaternion>))
  {
    types::TypeBase<geometry_msgs::msg::Quaternion> ret;
    ret.setValue(boost::any_cast<types::TypeBase<geometry_msgs::msg::Quaternion>>(v0).getValue() *
      boost::any_cast<types::TypeBase<geometry_msgs::msg::Quaternion>>(v1).getValue());
    return ret;
  }
  // (double value) * (double value)
  if (v0.type() == typeid(types::TypeBase<double>) &&
    v1.type() == typeid(types::TypeBase<double>))
  {
    types::TypeBase<double> ret;
    ret.setValue(boost::any_cast<types::TypeBase<double>>(v0).getValue() *
      boost::any_cast<types::TypeBase<double>>(v1).getValue());
    return ret;
  }
  // (int value) * (double value)
  if (v0.type() == typeid(types::TypeBase<int>) &&
    v1.type() == typeid(types::TypeBase<double>))
  {
    types::TypeBase<double> ret;
    ret.setValue(boost::any_cast<types::TypeBase<int>>(v0).getValue() *
      boost::any_cast<types::TypeBase<double>>(v1).getValue());
    return ret;
  }
  // (double value) * (int value)
  if (v0.type() == typeid(types::TypeBase<double>) &&
    v1.type() == typeid(types::TypeBase<int>))
  {
    types::TypeBase<double> ret;
    ret.setValue(boost::any_cast<types::TypeBase<double>>(v0).getValue() *
      boost::any_cast<types::TypeBase<int>>(v1).getValue());
    return ret;
  }
  // (int value) * (int value)
  if (v0.type() == typeid(types::TypeBase<int>) &&
    v1.type() == typeid(types::TypeBase<int>))
  {
    types::TypeBase<int> ret;
    ret.setValue(boost::any_cast<types::TypeBase<int>>(v0).getValue() *
      boost::any_cast<types::TypeBase<int>>(v1).getValue());
    return ret;
  }
  POLARIS_THROW_EVALUATION_ERROR(ast, "multiplication operators did not defined yet.");
}

boost::any Functions::division(std::shared_ptr<peg::Ast> ast)
{
  auto v0 = evaluate(ast->nodes[0]->name, ast->nodes[0]);
  auto v1 = evaluate(ast->nodes[2]->name, ast->nodes[2]);
  // (double value) * (double value)
  if (v0.type() == typeid(types::TypeBase<double>) &&
    v1.type() == typeid(types::TypeBase<double>))
  {
    types::TypeBase<double> ret;
    ret.setValue(boost::any_cast<types::TypeBase<double>>(v0).getValue() /
      boost::any_cast<types::TypeBase<double>>(v1).getValue());
    return ret;
  }
  // (int value) * (double value)
  if (v0.type() == typeid(types::TypeBase<int>) &&
    v1.type() == typeid(types::TypeBase<double>))
  {
    types::TypeBase<double> ret;
    ret.setValue(boost::any_cast<types::TypeBase<int>>(v0).getValue() /
      boost::any_cast<types::TypeBase<double>>(v1).getValue());
    return ret;
  }
  // (double value) * (int value)
  if (v0.type() == typeid(types::TypeBase<double>) &&
    v1.type() == typeid(types::TypeBase<int>))
  {
    types::TypeBase<double> ret;
    ret.setValue(boost::any_cast<types::TypeBase<double>>(v0).getValue() /
      boost::any_cast<types::TypeBase<int>>(v1).getValue());
    return ret;
  }
  // (int value) * (int value)
  if (v0.type() == typeid(types::TypeBase<int>) &&
    v1.type() == typeid(types::TypeBase<int>))
  {
    types::TypeBase<int> ret;
    ret.setValue(boost::any_cast<types::TypeBase<int>>(v0).getValue() /
      boost::any_cast<types::TypeBase<int>>(v1).getValue());
    return ret;
  }
  POLARIS_THROW_EVALUATION_ERROR(ast, "division operators did not defined yet.");
}

boost::any Functions::subtraction(std::shared_ptr<peg::Ast> ast)
{
  auto v0 = evaluate(ast->nodes[0]->name, ast->nodes[0]);
  auto v1 = evaluate(ast->nodes[2]->name, ast->nodes[2]);
  // (double value) - (double value)
  if (v0.type() == typeid(types::TypeBase<double>) &&
    v1.type() == typeid(types::TypeBase<double>))
  {
    types::TypeBase<double> ret;
    ret.setValue(boost::any_cast<types::TypeBase<double>>(v0).getValue() -
      boost::any_cast<types::TypeBase<double>>(v1).getValue());
    return ret;
  }
  // (int value) - (double value)
  if (v0.type() == typeid(types::TypeBase<int>) &&
    v1.type() == typeid(types::TypeBase<double>))
  {
    types::TypeBase<double> ret;
    ret.setValue(static_cast<double>(boost::any_cast<types::TypeBase<int>>(v0).getValue()) -
      boost::any_cast<types::TypeBase<double>>(v1).getValue());
    return ret;
  }
  // (double value) - (int value)
  if (v0.type() == typeid(types::TypeBase<double>) &&
    v1.type() == typeid(types::TypeBase<int>))
  {
    types::TypeBase<double> ret;
    ret.setValue(boost::any_cast<types::TypeBase<double>>(v0).getValue() -
      static_cast<double>(boost::any_cast<types::TypeBase<int>>(v1).getValue()));
    return ret;
  }
  // (int value) - (int value)
  if (v0.type() == typeid(types::TypeBase<double>) &&
    v1.type() == typeid(types::TypeBase<int>))
  {
    types::TypeBase<int> ret;
    ret.setValue(boost::any_cast<types::TypeBase<int>>(v0).getValue() -
      boost::any_cast<types::TypeBase<int>>(v1).getValue());
    return ret;
  }
  POLARIS_THROW_EVALUATION_ERROR(ast, "subcraction operators did not defined yet.");
}

boost::any Functions::addition(std::shared_ptr<peg::Ast> ast)
{
  auto v0 = evaluate(ast->nodes[0]->name, ast->nodes[0]);
  auto v1 = evaluate(ast->nodes[2]->name, ast->nodes[2]);
  // (double value) + (double value)
  if (v0.type() == typeid(types::TypeBase<double>) &&
    v1.type() == typeid(types::TypeBase<double>))
  {
    types::TypeBase<double> ret;
    ret.setValue(boost::any_cast<types::TypeBase<double>>(v0).getValue() +
      boost::any_cast<types::TypeBase<double>>(v1).getValue());
    return ret;
  }
  // (int value) + (double value)
  if (v0.type() == typeid(types::TypeBase<int>) &&
    v1.type() == typeid(types::TypeBase<double>))
  {
    types::TypeBase<double> ret;
    ret.setValue(static_cast<double>(boost::any_cast<types::TypeBase<int>>(v0).getValue()) +
      boost::any_cast<types::TypeBase<double>>(v1).getValue());
    return ret;
  }
  // (double value) + (int value)
  if (v0.type() == typeid(types::TypeBase<double>) &&
    v1.type() == typeid(types::TypeBase<int>))
  {
    types::TypeBase<double> ret;
    ret.setValue(boost::any_cast<types::TypeBase<double>>(v0).getValue() +
      static_cast<double>(boost::any_cast<types::TypeBase<int>>(v1).getValue()));
    return ret;
  }
  // (int value) + (int value)
  if (v0.type() == typeid(types::TypeBase<double>) &&
    v1.type() == typeid(types::TypeBase<int>))
  {
    types::TypeBase<int> ret;
    ret.setValue(boost::any_cast<types::TypeBase<int>>(v0).getValue() +
      boost::any_cast<types::TypeBase<int>>(v1).getValue());
    return ret;
  }
  POLARIS_THROW_EVALUATION_ERROR(ast, "addition operators did not defined yet.");
}
}  // namespace built_in_functions
}  // namespace polaris
