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

#ifndef POLARIS__PARSER__PARSER_HPP_
#define POLARIS__PARSER__PARSER_HPP_

#include <polaris/types/type_base.hpp>

#include <boost/any.hpp>

#include <peglib.h>

#include <memory>
#include <string>
#include <unordered_map>

namespace polaris
{
class Parser
{
public:
  Parser();
  bool evaluate(std::string line);
  template<typename T>
  bool getValue(std::string name, T & value) const
  {
    if (variables_.count(name) == 0) {
      return false;
    }
    boost::any variable = variables_.at(name);
    try {
      value = boost::any_cast<const types::TypeBase<T> &>(variable).getValue();
      return true;
    } catch (boost::bad_any_cast) {
      return false;
    }
    return false;
  }

private:
  boost::any evaluate(std::shared_ptr<peg::Ast> ast);
  std::unique_ptr<peg::parser> parser_ptr_;
  std::unordered_map<std::string, boost::any> variables_;
};
}  // namespace polaris

#endif  // POLARIS__PARSER__PARSER_HPP_
