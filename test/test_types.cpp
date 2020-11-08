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

TEST(types, pose_type_1)
{
  polaris::Parser parser;
  ASSERT_TRUE(parser.evaluate("let p = point(1,2,3);let a = pose(p,quaternion(0,0,0,1));"));
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

TEST(types, pose_type_2)
{
  polaris::Parser parser;
  ASSERT_TRUE(parser.evaluate("let q = quaternion(0,0,0,1);let a = pose(point(1,2,3),q);"));
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

TEST(types, string_0)
{
  std::string code = R"(let a = string("test");)";
  polaris::Parser parser;
  ASSERT_TRUE(parser.evaluate(code));
  const auto a = parser.getValue<std::string>("a");
  ASSERT_TRUE(a);
  ASSERT_STREQ(a.get().c_str(), "test");
}

TEST(types, string_1)
{
  std::string code = R"(let a = "test";)";
  polaris::Parser parser;
  ASSERT_TRUE(parser.evaluate(code));
  const auto a = parser.getValue<std::string>("a");
  ASSERT_TRUE(a);
  ASSERT_STREQ(a.get().c_str(), "test");
}

TEST(types, bool_0)
{
  std::string code = R"(let a = true;)";
  polaris::Parser parser;
  ASSERT_TRUE(parser.evaluate(code));
  const auto a = parser.getValue<bool>("a");
  ASSERT_TRUE(a);
  ASSERT_EQ(a.get(), true);
}

TEST(types, entity_0)
{
  std::string code =
    R"(let a = entity(pose(point(1,2,3),quaternion(0,0,0,1)), 
    ["bouy"], [point(0,1,2), point(2,3,4), point(3,2,3)]);)";
  polaris::Parser parser;
  ASSERT_TRUE(parser.evaluate(code));
  const auto a = parser.getValue<polaris::types::Entity>("a");
  ASSERT_TRUE(a);
  ASSERT_DOUBLE_EQ(a.get().pose.position.x, 1);
  ASSERT_DOUBLE_EQ(a.get().pose.position.y, 2);
  ASSERT_DOUBLE_EQ(a.get().pose.position.z, 3);
  ASSERT_EQ(a.get().type.size(), static_cast<size_t>(1));
  ASSERT_STREQ(a.get().type[0].c_str(), "bouy");
  ASSERT_EQ(a.get().polygon.size(), static_cast<size_t>(3));
  ASSERT_DOUBLE_EQ(a.get().polygon[0].x, 0);
  ASSERT_DOUBLE_EQ(a.get().polygon[0].y, 1);
  ASSERT_DOUBLE_EQ(a.get().polygon[0].z, 2);
  ASSERT_DOUBLE_EQ(a.get().polygon[1].x, 2);
  ASSERT_DOUBLE_EQ(a.get().polygon[1].y, 3);
  ASSERT_DOUBLE_EQ(a.get().polygon[1].z, 4);
  ASSERT_DOUBLE_EQ(a.get().polygon[2].x, 3);
  ASSERT_DOUBLE_EQ(a.get().polygon[2].y, 2);
  ASSERT_DOUBLE_EQ(a.get().polygon[2].z, 3);
}

TEST(types, array_0)
{
  std::string code = R"(let a = [1,2,3];)";
  polaris::Parser parser;
  ASSERT_TRUE(parser.evaluate(code));
  const auto a = parser.getValue<std::vector<int>>("a");
  ASSERT_TRUE(a);
  ASSERT_EQ(a.get().size(), static_cast<size_t>(3));
  ASSERT_EQ(a.get()[0], 1);
  ASSERT_EQ(a.get()[1], 2);
  ASSERT_EQ(a.get()[2], 3);
}

TEST(types, array_1)
{
  std::string code = R"(let a = [1.0,2.9,3.0,12.0];)";
  polaris::Parser parser;
  ASSERT_TRUE(parser.evaluate(code));
  const auto a = parser.getValue<std::vector<double>>("a");
  ASSERT_TRUE(a);
  ASSERT_DOUBLE_EQ(a.get().size(), static_cast<size_t>(4));
  ASSERT_DOUBLE_EQ(a.get()[0], 1.0);
  ASSERT_DOUBLE_EQ(a.get()[1], 2.9);
  ASSERT_DOUBLE_EQ(a.get()[2], 3.0);
  ASSERT_DOUBLE_EQ(a.get()[3], 12.0);
}

TEST(types, array_2)
{
  std::string code = R"(let a = ["a","b"];)";
  polaris::Parser parser;
  ASSERT_TRUE(parser.evaluate(code));
  const auto a = parser.getValue<std::vector<std::string>>("a");
  ASSERT_TRUE(a);
  ASSERT_DOUBLE_EQ(a.get().size(), static_cast<size_t>(2));
  ASSERT_STREQ(a.get()[0].c_str(), "a");
  ASSERT_STREQ(a.get()[1].c_str(), "b");
}

TEST(types, array_3)
{
  std::string code = R"(let a=[point(1,2,3), point(1,2,5)];)";
  polaris::Parser parser;
  ASSERT_TRUE(parser.evaluate(code));
  const auto a = parser.getValue<std::vector<geometry_msgs::msg::Point>>("a");
  ASSERT_TRUE(a);
  ASSERT_EQ(a.get().size(), static_cast<size_t>(2));
  ASSERT_DOUBLE_EQ(a.get()[0].x, 1.0);
  ASSERT_DOUBLE_EQ(a.get()[0].y, 2.0);
  ASSERT_DOUBLE_EQ(a.get()[0].z, 3.0);
  ASSERT_DOUBLE_EQ(a.get()[1].x, 1.0);
  ASSERT_DOUBLE_EQ(a.get()[1].y, 2.0);
  ASSERT_DOUBLE_EQ(a.get()[1].z, 5.0);
}

TEST(types, array_4)
{
  std::string code = R"(let p = point(1,2,3);let a=[p, point(1,2,5)];)";
  polaris::Parser parser;
  ASSERT_TRUE(parser.evaluate(code));
  const auto a = parser.getValue<std::vector<geometry_msgs::msg::Point>>("a");
  ASSERT_TRUE(a);
  ASSERT_EQ(a.get().size(), static_cast<size_t>(2));
  ASSERT_DOUBLE_EQ(a.get()[0].x, 1.0);
  ASSERT_DOUBLE_EQ(a.get()[0].y, 2.0);
  ASSERT_DOUBLE_EQ(a.get()[0].z, 3.0);
  ASSERT_DOUBLE_EQ(a.get()[1].x, 1.0);
  ASSERT_DOUBLE_EQ(a.get()[1].y, 2.0);
  ASSERT_DOUBLE_EQ(a.get()[1].z, 5.0);
}

TEST(types, array_5)
{
  std::string code =
    R"(
      let a = entity(pose(point(1,2,3),quaternion(0,0,0,1)),
      ["bouy"], [point(0,1,2), point(2,3,4), point(3,2,3)]);
      let b = entity(pose(point(1,2,3),quaternion(0,0,0,1)), 
      ["bouy"], [point(0,1,2), point(2,3,4), point(3,2,3)]);
      let c = [a,b];
    )";
  polaris::Parser parser;
  ASSERT_TRUE(parser.evaluate(code));
  const auto c = parser.getValue<std::vector<polaris::types::Entity>>("c");
  ASSERT_TRUE(c);
  ASSERT_EQ(c.get().size(), static_cast<size_t>(2));
  const auto a = parser.getValue<polaris::types::Entity>("a");
  ASSERT_TRUE(a);
  ASSERT_DOUBLE_EQ(a.get().pose.position.x, 1);
  ASSERT_DOUBLE_EQ(a.get().pose.position.y, 2);
  ASSERT_DOUBLE_EQ(a.get().pose.position.z, 3);
  ASSERT_EQ(a.get().type.size(), static_cast<size_t>(1));
  ASSERT_STREQ(a.get().type[0].c_str(), "bouy");
  ASSERT_EQ(a.get().polygon.size(), static_cast<size_t>(3));
  ASSERT_DOUBLE_EQ(a.get().polygon[0].x, 0);
  ASSERT_DOUBLE_EQ(a.get().polygon[0].y, 1);
  ASSERT_DOUBLE_EQ(a.get().polygon[0].z, 2);
  ASSERT_DOUBLE_EQ(a.get().polygon[1].x, 2);
  ASSERT_DOUBLE_EQ(a.get().polygon[1].y, 3);
  ASSERT_DOUBLE_EQ(a.get().polygon[1].z, 4);
  ASSERT_DOUBLE_EQ(a.get().polygon[2].x, 3);
  ASSERT_DOUBLE_EQ(a.get().polygon[2].y, 2);
  ASSERT_DOUBLE_EQ(a.get().polygon[2].z, 3);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
