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

#include <polaris/grammar/grammar.hpp>
#include <polaris/types/type_base.hpp>
#include <polaris/parser/parser.hpp>

#include <boost/optional.hpp>

#include <memory>
#include <string>
#include <vector>

namespace polaris
{
Parser::Parser()
{
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
  if (ast->name == "ASSIGNMENT") {
    auto symbol = ast->nodes[0]->token;
    auto value = evaluate(ast->nodes[1]);
    variables_[symbol] = value;
    return value;
  }
  if (ast->name == "DOUBLE") {
    types::TypeBase<double> double_value;
    double_value.setValue(stod(ast->token));
    return double_value;
  }
  return boost::none;
  // throw std::runtime_error("ast name " + ast->name + " does not implemented.");
}
}  // namespace polaris
