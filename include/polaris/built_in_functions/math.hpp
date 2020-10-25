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

#ifndef POLARIS__BUILT_IN_FUNCTIONS__MATH_HPP_
#define POLARIS__BUILT_IN_FUNCTIONS__MATH_HPP_

#include <peglib.h>

#include <polaris/types/type_base.hpp>
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
namespace math
{
boost::any construct_integer(std::shared_ptr<peg::Ast> ast);
boost::any construct_double(std::shared_ptr<peg::Ast> ast);
boost::any construct_quaternion(std::shared_ptr<peg::Ast> ast);
}  // namespace math
}  // namespace built_in_functions
}  // namespace polaris
#endif  // POLARIS__BUILT_IN_FUNCTIONS__MATH_HPP_
