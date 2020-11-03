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

#include <polaris/exception.hpp>
#include <polaris/grammar/grammar.hpp>
#include <polaris/built_in_functions/functions.hpp>
#include <polaris/types/type_base.hpp>
#include <polaris/parser/parser.hpp>

#include <boost/optional.hpp>

#include <memory>
#include <string>
#include <vector>
#include <iostream>


namespace polaris
{
Parser::Parser(bool verbose)
{
  verbose_ = verbose;
  std::string grammar = polaris::grammar;
  parser_ptr_ = std::make_unique<peg::parser>(grammar.c_str());
  parser_ptr_->enable_ast();
}

bool Parser::evaluate(std::string line)
{
  variables_.clear();
  std::shared_ptr<peg::Ast> ast_ptr;
  auto ret = parser_ptr_->parse(line.c_str(), ast_ptr);
  if (!ret) {
    return false;
  }
  ast_ptr = peg::AstOptimizer(true).optimize(ast_ptr);
  evaluate(ast_ptr);
  return true;
}

boost::any Parser::evaluate(std::shared_ptr<peg::Ast> ast)
{
  if (verbose_) {
    std::cout << peg::ast_to_s(ast) << std::endl;
  }
  if (ast->name == "STATEMENTS") {
    for (size_t i = 0; i < ast->nodes.size(); i++) {
      evaluate(ast->nodes[i]);
      functions_.setVariables(variables_);
    }
  }
  if (ast->nodes.size() >= 3) {
    if (ast->nodes[1]->name == "INFIX_OPE") {
      return functions_.evaluate(ast->nodes[1]->token, ast);
    }
  }
  if (ast->name == "ASSIGNMENT") {
    auto symbol = ast->nodes[0]->token;
    auto value = evaluate(ast->nodes[1]);
    variables_[symbol] = value;
    return value;
  }
  if (ast->name == "DOUBLE") {
    return functions_.evaluate("double", ast);
  }
  if (ast->name == "INTEGER") {
    return functions_.evaluate("integer", ast);
  }
  if (ast->name == "STRING") {
    return functions_.evaluate("string", ast);
  }
  if (ast->name == "CALL") {
    auto ret = functions_.evaluate(ast->nodes[0]->token, ast->nodes[1]);
    if (ret.type() == typeid(boost::none)) {
      if (variables_.count(ast->nodes[0]->token) != 0) {
        return variables_[ast->nodes[0]->token];
      }
      return boost::none;
    }
    return ret;
  }
  return boost::none;
}
}  // namespace polaris
