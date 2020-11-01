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

#ifndef POLARIS__EXCEPTION_HPP_
#define POLARIS__EXCEPTION_HPP_

#include <peglib.h>

#include <memory>
#include <exception>
#include <iostream>
#include <sstream>
#include <string>

namespace polaris
{
class EvaluationError : public std::runtime_error
{
public:
  EvaluationError(
    std::shared_ptr<peg::Ast> ast, std::string description, const char * file,
    int line)
  : std::runtime_error(("\nast => \n" + peg::ast_to_s(ast) +
      "\ndescription => " + description +
      "\nfile => " + file +
      "\nline => " + std::to_string(line)
      ).c_str()),
    ast_string(peg::ast_to_s(ast)), description(description)
  {}
  const std::string ast_string;
  const std::string description;
};
}  // namespace polaris

#define POLARIS_THROW_EVALUATION_ERROR(ast, description) throw polaris::EvaluationError(ast, \
    description, \
    __FILE__, \
    __LINE__);

#endif  // POLARIS__EXCEPTION_HPP_
