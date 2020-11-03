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

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <vector>
#include <string>

TEST(types, empty)
{
  polaris::Parser parser;
  ASSERT_TRUE(parser.evaluate(""));
}

TEST(types, double_type)
{
  polaris::Parser parser;
  ASSERT_TRUE(parser.evaluate("let a = 1.0;"));
  const auto a = parser.getValue<double>("a");
  ASSERT_TRUE(a);
  ASSERT_DOUBLE_EQ(a.get(), 1.0);
  ASSERT_TRUE(parser.evaluate("1;"));
  ASSERT_TRUE(parser.evaluate("1.5;"));
  ASSERT_TRUE(parser.evaluate("-1.2;"));
  ASSERT_FALSE(parser.evaluate("-1.2a;"));
  ASSERT_FALSE(parser.evaluate("a3;"));
  ASSERT_TRUE(parser.evaluate("a;3;"));
}

TEST(types, quaternion_type_0)
{
  polaris::Parser parser;
  ASSERT_TRUE(parser.evaluate("let a = quaternion(0.0,0.0,0.0,1.0);"));
  const auto a = parser.getValue<geometry_msgs::msg::Quaternion>("a");
  ASSERT_TRUE(a);
  ASSERT_DOUBLE_EQ(a.get().x, 0.0);
  ASSERT_DOUBLE_EQ(a.get().y, 0.0);
  ASSERT_DOUBLE_EQ(a.get().z, 0.0);
  ASSERT_DOUBLE_EQ(a.get().w, 1.0);
}

TEST(types, quaternion_type_1)
{
  polaris::Parser parser;
  ASSERT_TRUE(parser.evaluate("let w = 1.0;let a = quaternion(double(0.0),0,0.0,w);"));
  const auto a = parser.getValue<geometry_msgs::msg::Quaternion>("a");
  ASSERT_TRUE(a);
  ASSERT_DOUBLE_EQ(a.get().x, 0.0);
  ASSERT_DOUBLE_EQ(a.get().y, 0.0);
  ASSERT_DOUBLE_EQ(a.get().z, 0.0);
  ASSERT_DOUBLE_EQ(a.get().w, 1.0);
  const auto w = parser.getValue<double>("w");
  ASSERT_TRUE(w);
  ASSERT_DOUBLE_EQ(w.get(), 1.0);
  const auto x = parser.getValue<double>("x");
  ASSERT_FALSE(x);
}

TEST(types, point_type_0)
{
  polaris::Parser parser;
  ASSERT_TRUE(parser.evaluate("let a = point(0.0,0.0,1.0);"));
  const auto a = parser.getValue<geometry_msgs::msg::Point>("a");
  ASSERT_TRUE(a);
  ASSERT_DOUBLE_EQ(a.get().x, 0.0);
  ASSERT_DOUBLE_EQ(a.get().y, 0.0);
  ASSERT_DOUBLE_EQ(a.get().z, 1.0);
}

TEST(types, pose_type_0)
{
  polaris::Parser parser;
  ASSERT_TRUE(parser.evaluate("let a = pose(point(1,2,3),quaternion(0,0,0,1));"));
  const auto a = parser.getValue<geometry_msgs::msg::Pose>("a");
  ASSERT_TRUE(a);
  ASSERT_DOUBLE_EQ(a.get().position.x, 1.0);
  ASSERT_DOUBLE_EQ(a.get().position.y, 2.0);
  ASSERT_DOUBLE_EQ(a.get().position.z, 3.0);
  ASSERT_DOUBLE_EQ(a.get().orientation.x, 0.0);
  ASSERT_DOUBLE_EQ(a.get().orientation.y, 0.0);
  ASSERT_DOUBLE_EQ(a.get().orientation.z, 0.0);
  ASSERT_DOUBLE_EQ(a.get().orientation.w, 1.0);
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
