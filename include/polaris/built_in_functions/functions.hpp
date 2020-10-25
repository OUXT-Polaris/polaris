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

#include <polaris/built_in_functions/math.hpp>

#include <peglib.h>

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
    functions_.insert(std::make_pair("double", math::construct_double));
    functions_.insert(std::make_pair("quaternion", math::construct_quaternion));
  }
  boost::any evaluate(std::string function, std::shared_ptr<peg::Ast> ast)
  {
    if (functions_.count(function) == 0) {
      return boost::none;
    }
    return functions_[function](ast);
  }

private:
  std::unordered_map<std::string,
    std::function<boost::any(std::shared_ptr<peg::Ast> ast)>> functions_;
};
// {"quaternion", math::construct_quaternion}
}  // namespace built_in_functions
}  // namespace polaris

#endif  // POLARIS__BUILT_IN_FUNCTIONS__FUNCTIONS_HPP_