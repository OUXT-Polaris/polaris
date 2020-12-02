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

#include <polaris/types/type_base.hpp>
#include <polaris/types/entity.hpp>
#include <polaris/types/task/task.hpp>
#include <polaris/exception.hpp>

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
/**
 * @brief Function class
 */
class Functions
{
public:
  /**
   * @brief Construct a new Functions object
   */
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
    functions_.insert(std::make_pair("string",
      std::bind(&Functions::constructString, this, std::placeholders::_1)));
    functions_.insert(std::make_pair("bool",
      std::bind(&Functions::constructBoolean, this, std::placeholders::_1)));
    functions_.insert(std::make_pair("boolean",
      std::bind(&Functions::constructBoolean, this, std::placeholders::_1)));
    functions_.insert(std::make_pair("BOOLEAN",
      std::bind(&Functions::constructBoolean, this, std::placeholders::_1)));
    functions_.insert(std::make_pair("STRING",
      std::bind(&Functions::constructString, this, std::placeholders::_1)));
    functions_.insert(std::make_pair("array",
      std::bind(&Functions::constructArray, this, std::placeholders::_1)));
    functions_.insert(std::make_pair("ARRAY",
      std::bind(&Functions::constructArray, this, std::placeholders::_1)));
    functions_.insert(std::make_pair("quaternion",
      std::bind(&Functions::constructQuaternion, this, std::placeholders::_1)));
    functions_.insert(std::make_pair("rpy",
      std::bind(&Functions::constructQuaternionFromRpy, this, std::placeholders::_1)));
    functions_.insert(std::make_pair("point",
      std::bind(&Functions::constructPoint, this, std::placeholders::_1)));
    functions_.insert(std::make_pair("pose",
      std::bind(&Functions::constructPose, this, std::placeholders::_1)));
    functions_.insert(std::make_pair("entity",
      std::bind(&Functions::constructEntity, this, std::placeholders::_1)));
    functions_.insert(std::make_pair("+",
      std::bind(&Functions::addition, this, std::placeholders::_1)));
    functions_.insert(std::make_pair("-",
      std::bind(&Functions::subtraction, this, std::placeholders::_1)));
    functions_.insert(std::make_pair("*",
      std::bind(&Functions::multiplication, this, std::placeholders::_1)));
    functions_.insert(std::make_pair("/",
      std::bind(&Functions::division, this, std::placeholders::_1)));
    functions_.insert(std::make_pair("IDENTIFIER",
      std::bind(&Functions::fetchVariable, this, std::placeholders::_1)));
  }
  /**
   * @brief evaluate abstract syntax tree
   * @param function the name of the function
   * @param ast shared pointer to the abstract syntax tree class
   * @return boost::any interpretation result of the abstract syntax tree
   */
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

  /**
   * @brief Set the variable object to the variable store
   * @param variables variable
   */
  void setVariables(std::unordered_map<std::string, boost::any> variables)
  {
    variables_ = variables;
  }

private:
  /**
   * @brief local variables store
   */
  std::unordered_map<std::string, boost::any> variables_;
  /**
   * @brief key/value data of the name of the function and it's arguments
   */
  std::unordered_map<std::string,
    std::function<boost::any(std::shared_ptr<peg::Ast> ast)>> functions_;
  /**
   * @brief constructor for boolean type variable
   * @param ast shared pointer to the abstract syntax tree class
   * @return boost::any constracted boolean type variable
   */
  boost::any constructBoolean(std::shared_ptr<peg::Ast> ast);
  /**
   * @brief constructor for string type variable
   * @param ast shared pointer to the abstract syntax tree class
   * @return boost::any constructed string type variable
   */
  boost::any constructString(std::shared_ptr<peg::Ast> ast);
  /**
   * @brief constructor for integer type variable
   * @param ast shared pointer to the abstract syntax tree class
   * @return boost::any constructed integer type variable
   */
  boost::any constructInteger(std::shared_ptr<peg::Ast> ast);
  /**
   * @brief constructor for double type variable
   * @param ast shared pointer to the abstract syntax tree class
   * @return boost::any constructed double type variable
   */
  boost::any constructDouble(std::shared_ptr<peg::Ast> ast);
  /**
   * @brief constructor for quaternion type variable
   * @param ast shared pointer to the abstract syntax tree class
   * @return boost::any constructed quaternion type variable
   */
  boost::any constructQuaternion(std::shared_ptr<peg::Ast> ast);
  /**
   * @brief constructor for quaternion type variable from RPY variable
   * @param ast shared pointer to the abstract syntax tree class
   * @return boost::any constructed quaternion type variable
   */
  boost::any constructQuaternionFromRpy(std::shared_ptr<peg::Ast> ast);
  /**
   * @brief constructor for point type variable
   * @param ast shared pointer to the abstract syntax tree class
   * @return boost::any constructed point type variable
   */
  boost::any constructPoint(std::shared_ptr<peg::Ast> ast);
  /**
   * @brief constructor for pose type variable
   * @param ast shared pointer to the abstract syntax tree class
   * @return boost::any constructed pose type variable
   */
  boost::any constructPose(std::shared_ptr<peg::Ast> ast);
  /**
   * @brief constructor for array type variable
   * @param ast shared pointer to the abstract syntax tree class
   * @return boost::any constructed array type variable
   */
  boost::any constructArray(std::shared_ptr<peg::Ast> ast);
  /**
   * @brief constructor for entity type variable
   * @param ast shared pointer to the abstract syntax tree class
   * @return boost::any constructed entity type variable
   */
  boost::any constructEntity(std::shared_ptr<peg::Ast> ast);
  /**
   * @brief addition operator
   * @param ast shared pointer to the abstract syntax tree class
   * @return boost::any result of the addition operation
   */
  boost::any addition(std::shared_ptr<peg::Ast> ast);
  /**
   * @brief subtraction operator
   * @param ast shared pointer to the abstract syntax tree class
   * @return boost::any result of the subtraction operation
   */
  boost::any subtraction(std::shared_ptr<peg::Ast> ast);
  /**
   * @brief multiplication operator
   * @param ast shared pointer to the abstract syntax tree class
   * @return boost::any result of the multiplication operation
   */
  boost::any multiplication(std::shared_ptr<peg::Ast> ast);
  /**
   * @brief division operator
   * @param ast shared pointer to the abstract syntax tree class
   * @return boost::any result of the division operation
   */
  boost::any division(std::shared_ptr<peg::Ast> ast);
  /**
   * @brief function fetching variables from variable store
   * @param ast shared pointer to the abstract syntax tree class
   * @return boost::any stored variable
   */
  boost::any fetchVariable(std::shared_ptr<peg::Ast> ast);
};
}  // namespace built_in_functions
}  // namespace polaris

#endif  // POLARIS__BUILT_IN_FUNCTIONS__FUNCTIONS_HPP_
