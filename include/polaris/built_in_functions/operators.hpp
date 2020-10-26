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

#ifndef POLARIS__BUILT_IN_FUNCTIONS__OPERATORS_HPP_
#define POLARIS__BUILT_IN_FUNCTIONS__OPERATORS_HPP_

#include <peglib.h>

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
class Operators
{
public:
  Operators()
  {
    /*
    functions_.insert(std::make_pair("integer",
      std::bind(&Functions::constructInteger, this, std::placeholders::_1)));
    functions_.insert(std::make_pair("double",
      std::bind(&Functions::constructDouble, this, std::placeholders::_1)));
    functions_.insert(std::make_pair("quaternion",
      std::bind(&Functions::constructDuaternion, this, std::placeholders::_1)));
    */
  }
  boost::any evaluate(std::string function, std::shared_ptr<peg::Ast> ast)
  {
    /*
    if (functions_.count(function) == 0) {
      return boost::none;
    }
    return functions_[function](ast);
    */
  }

  void setVariables(std::unordered_map<std::string, boost::any> variables)
  {
    variables_ = variables;
  }

private:
  std::unordered_map<std::string, boost::any> variables_;
  std::unordered_map<std::string,
    std::function<boost::any(std::shared_ptr<peg::Ast> ast)>> functions_;
};
}  // namespace built_in_functions
}  // namespace polaris

#endif  // POLARIS__BUILT_IN_FUNCTIONS__OPERATORS_HPP_
