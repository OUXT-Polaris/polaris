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

#ifndef POLARIS__TYPES__TYPE_BASE_HPP_
#define POLARIS__TYPES__TYPE_BASE_HPP_

#include <string>

namespace polaris
{
namespace types
{
template<class T>
class TypeBase
{
public:
  TypeBase(
    const std::string & grammar)
  : grammar(grammar)
  {}
  const std::string grammar;
  const T getValue() const {return value;}

private:
  T value;
};
}  // namespace types
}  // namespace polaris

#endif  // POLARIS__TYPES__TYPE_BASE_HPP_
