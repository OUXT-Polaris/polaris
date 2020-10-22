cmake_minimum_required(VERSION 3.5)
project(polaris)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(peglib_vendor REQUIRED)

include_directories(include)

add_library(parser
  src/parser/parser.cpp
)
ament_target_dependencies(parser
  geometry_msgs
  peglib_vendor)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(peglib_vendor REQUIRED)

install(
  DIRECTORY "include/"
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_parser test/test_parser.cpp)
  target_link_libraries(test_parser parser)
endif()

ament_export_include_directories(include)
ament_package()
