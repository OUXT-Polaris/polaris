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

add_library(polaris SHARED
  src/parser/parser.cpp
  src/built_in_functions/functions.cpp
)
ament_target_dependencies(polaris
  geometry_msgs
  peglib_vendor)
target_link_libraries(polaris
  Threads::Threads
)
ament_export_interfaces(export_polaris HAS_LIBRARY_TARGET)
ament_export_libraries(polaris)

add_executable(example
  src/example/example.cpp
)
ament_target_dependencies(example
  geometry_msgs
  peglib_vendor)
target_link_libraries(example
  polaris
  Threads::Threads
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(peglib_vendor REQUIRED)

install(
  DIRECTORY "include/"
  DESTINATION include
)
install(TARGETS
  example
  DESTINATION lib/polaris
)
install(TARGETS polaris
EXPORT export_polaris
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_types test/test_types.cpp)
  target_link_libraries(test_types polaris)

  ament_add_gtest(test_operator test/test_operator.cpp)
  target_link_libraries(test_operator polaris)
endif()

ament_export_include_directories(include)
ament_package()
