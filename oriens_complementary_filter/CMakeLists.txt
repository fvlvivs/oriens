cmake_minimum_required(VERSION 3.8)
project(oriens_complementary_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(oriens_core REQUIRED)
find_package(oriens_msgs REQUIRED)

set(dependencies
  rclcpp
  Eigen3
  oriens_core
  oriens_msgs
)

include_directories(include)

add_executable(complementary_filter
  src/complementary_filter_node.cpp
  src/complementary_filter.cpp
)

ament_target_dependencies(complementary_filter ${dependencies})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS complementary_filter DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
