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

#ifndef POLARIS__BUILT_IN_FUNCTIONS__FUNCTIONS_HPP_
#define POLARIS__BUILT_IN_FUNCTIONS__FUNCTIONS_HPP_

#include <polaris/exception.hpp>
#include <polaris/types/type_base.hpp>

#include <peglib.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <boost/optional.hpp>
#include <boost/any.hpp>

#include <functional>
#include <unordered_map>
#include <memory>
#include <string>
#include <utility>

namespace polaris
{
namespace built_in_functions
{
class Functions
{
public:
  Functions()
  {
    functions_.insert(std::make_pair("integer",
      std::bind(&Functions::constructInteger, this, std::placeholders::_1)));
    functions_.insert(std::make_pair("INTEGER",
      std::bind(&Functions::constructInteger, this, std::placeholders::_1)));
    functions_.insert(std::make_pair("double",
      std::bind(&Functions::constructDouble, this, std::placeholders::_1)));
    functions_.insert(std::make_pair("DOUBLE",
      std::bind(&Functions::constructDouble, this, std::placeholders::_1)));
    functions_.insert(std::make_pair("STRING",
      std::bind(&Functions::constructString, this, std::placeholders::_1)));
    functions_.insert(std::make_pair("quaternion",
      std::bind(&Functions::constructQuaternion, this, std::placeholders::_1)));
    functions_.insert(std::make_pair("rpy",
      std::bind(&Functions::constructQuaternionFromRpy, this, std::placeholders::_1)));
    functions_.insert(std::make_pair("point",
      std::bind(&Functions::constructPoint, this, std::placeholders::_1)));
    functions_.insert(std::make_pair("pose",
      std::bind(&Functions::constructPose, this, std::placeholders::_1)));
    functions_.insert(std::make_pair("+",
      std::bind(&Functions::addition, this, std::placeholders::_1)));
    functions_.insert(std::make_pair("-",
      std::bind(&Functions::subtraction, this, std::placeholders::_1)));
    functions_.insert(std::make_pair("*",
      std::bind(&Functions::multiplication, this, std::placeholders::_1)));
    functions_.insert(std::make_pair("/",
      std::bind(&Functions::division, this, std::placeholders::_1)));
  }
  boost::any evaluate(std::string function, std::shared_ptr<peg::Ast> ast)
  {
    if (ast->name == "PREFIX_EXPR") {
      if (ast->nodes[0]->token == "-") {
        auto ret = evaluate(ast->nodes[1]->name, ast->nodes[1]);
        if (ret.type() == typeid(boost::none)) {
          return boost::none;
        }
        if (ret.type() == typeid(types::TypeBase<double>)) {
          types::TypeBase<double> double_value = boost::any_cast<types::TypeBase<double>>(ret);
          double_value.setValue(-1 * double_value.getValue());
          return double_value;
        }
        if (ret.type() == typeid(types::TypeBase<int>)) {
          types::TypeBase<int> int_value = boost::any_cast<types::TypeBase<int>>(ret);
          int_value.setValue(-1 * int_value.getValue());
          return int_value;
        }
      }
    }
    if (ast->name == "CALL") {
      auto ret = evaluate(ast->nodes[0]->token, ast->nodes[1]);
      if (ret.type() == typeid(boost::none)) {
        if (variables_.count(ast->nodes[0]->token) != 0) {
          return variables_[ast->nodes[0]->token];
        }
        return boost::none;
      }
      return ret;
    }
    if (functions_.count(function) == 0) {
      POLARIS_THROW_EVALUATION_ERROR(ast, "function did not defined yet.");
    }
    return functions_[function](ast);
  }

  void setVariables(std::unordered_map<std::string, boost::any> variables)
  {
    variables_ = variables;
  }

private:
  std::unordered_map<std::string, boost::any> variables_;
  std::unordered_map<std::string,
    std::function<boost::any(std::shared_ptr<peg::Ast> ast)>> functions_;
  boost::any constructString(std::shared_ptr<peg::Ast> ast);
  boost::any constructInteger(std::shared_ptr<peg::Ast> ast);
  boost::any constructDouble(std::shared_ptr<peg::Ast> ast);
  boost::any constructQuaternion(std::shared_ptr<peg::Ast> ast);
  boost::any constructQuaternionFromRpy(std::shared_ptr<peg::Ast> ast);
  boost::any constructPoint(std::shared_ptr<peg::Ast> ast);
  boost::any constructPose(std::shared_ptr<peg::Ast> ast);
  boost::any addition(std::shared_ptr<peg::Ast> ast);
  boost::any subtraction(std::shared_ptr<peg::Ast> ast);
  boost::any multiplication(std::shared_ptr<peg::Ast> ast);
  boost::any division(std::shared_ptr<peg::Ast> ast);
};
}  // namespace built_in_functions
}  // namespace polaris

#endif  // POLARIS__BUILT_IN_FUNCTIONS__FUNCTIONS_HPP_
