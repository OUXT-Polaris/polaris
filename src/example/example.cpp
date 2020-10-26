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

#include <polaris/parser/parser.hpp>

#include <iostream>

int main()
{
  polaris::Parser parser(true);
  /*
  if (parser.evaluate("let a = 1.0;")) {
    std::cout << "evaluate succeced" << std::endl;
    auto a_value = parser.getValue<double>("a");
    if (a_value) {
      std::cout << "a = " << a_value.get() << std::endl;
    }
  } else {
    std::cout << "evaluate failed" << std::endl;
  }
  if (parser.evaluate("let w = 1.0;let a = quaternion(double(0.0),0,0.0,w);")) {
    std::cout << "evaluate succeced" << std::endl;
    auto a_value = parser.getValue<geometry_msgs::msg::Quaternion>("a");
    if (a_value) {
      std::cout << "a.x = " << a_value.get().x << std::endl;
      std::cout << "a.y = " << a_value.get().y << std::endl;
      std::cout << "a.z = " << a_value.get().z << std::endl;
      std::cout << "a.w = " << a_value.get().w << std::endl;
    }
  }
  */
  if (parser.evaluate("1.0+3.0;")) {
    std::cout << "evaluate succeced" << std::endl;
  }
}
