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

#include <gtest/gtest.h>

#include <polaris/parser/parser.hpp>

#include <geometry_msgs/msg/quaternion.hpp>

#include <vector>
#include <string>

TEST(operator, addition0)
{
  polaris::Parser parser;
  ASSERT_TRUE(parser.evaluate("let a = 1.0 + 3.0;"));
  const auto a = parser.getValue<double>("a");
  ASSERT_TRUE(a);
  ASSERT_DOUBLE_EQ(a.get(), 4.0);
}

TEST(operator, addition1)
{
  polaris::Parser parser;
  ASSERT_TRUE(parser.evaluate("let a = 1 + 3.0;"));
  const auto a = parser.getValue<double>("a");
  ASSERT_TRUE(a);
  ASSERT_DOUBLE_EQ(a.get(), 4.0);
}

TEST(operator, addition2)
{
  polaris::Parser parser;
  ASSERT_TRUE(parser.evaluate("let a = 1 + double(3.0);"));
  const auto a = parser.getValue<double>("a");
  ASSERT_TRUE(a);
  ASSERT_DOUBLE_EQ(a.get(), 4.0);
}

TEST(operator, addition3)
{
  polaris::Parser parser;
  ASSERT_TRUE(parser.evaluate("let a = -1 + double(3.0);"));
  const auto a = parser.getValue<double>("a");
  ASSERT_TRUE(a);
  ASSERT_DOUBLE_EQ(a.get(), 2.0);
}

TEST(operator, subtraction0)
{
  polaris::Parser parser;
  ASSERT_TRUE(parser.evaluate("let a = 1.0 - 3.0;"));
  const auto a = parser.getValue<double>("a");
  ASSERT_TRUE(a);
  ASSERT_DOUBLE_EQ(a.get(), -2.0);
}

TEST(operator, subtraction1)
{
  polaris::Parser parser;
  ASSERT_TRUE(parser.evaluate("let a = 1 - 3.0;"));
  const auto a = parser.getValue<double>("a");
  ASSERT_TRUE(a);
  ASSERT_DOUBLE_EQ(a.get(), -2.0);
}

TEST(operator, subtraction2)
{
  polaris::Parser parser;
  ASSERT_TRUE(parser.evaluate("let a = 1 - double(3.0);"));
  const auto a = parser.getValue<double>("a");
  ASSERT_TRUE(a);
  ASSERT_DOUBLE_EQ(a.get(), -2.0);
}

TEST(operator, subtraction3)
{
  polaris::Parser parser;
  ASSERT_TRUE(parser.evaluate("let a = -1 - double(3.0);"));
  const auto a = parser.getValue<double>("a");
  ASSERT_TRUE(a);
  ASSERT_DOUBLE_EQ(a.get(), -4.0);
}

TEST(operator, multiplication0)
{
  polaris::Parser parser;
  ASSERT_TRUE(parser.evaluate("let a = 1.0 * 3.0;"));
  const auto a = parser.getValue<double>("a");
  ASSERT_TRUE(a);
  ASSERT_DOUBLE_EQ(a.get(), 3.0);
}

TEST(operator, multiplication1)
{
  polaris::Parser parser;
  ASSERT_TRUE(parser.evaluate("let a = 1 * 3.0;"));
  const auto a = parser.getValue<double>("a");
  ASSERT_TRUE(a);
  ASSERT_DOUBLE_EQ(a.get(), 3.0);
}

TEST(operator, multiplication2)
{
  polaris::Parser parser;
  ASSERT_TRUE(parser.evaluate("let a = 1 * double(3.0);"));
  const auto a = parser.getValue<double>("a");
  ASSERT_TRUE(a);
  ASSERT_DOUBLE_EQ(a.get(), 3.0);
}

TEST(operator, multiplication3)
{
  polaris::Parser parser;
  ASSERT_TRUE(parser.evaluate("let a = -1 * double(3.0);"));
  const auto a = parser.getValue<double>("a");
  ASSERT_TRUE(a);
  ASSERT_DOUBLE_EQ(a.get(), -3.0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
