cmake_minimum_required(VERSION 3.8)
project(oriens_support)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(oriens_msgs REQUIRED)

set(dependencies
  rclcpp
  sensor_msgs
  geometry_msgs
  oriens_msgs
)

include_directories(include)

add_executable(serial_reader
src/serial_reader_node.cpp
src/serial_reader.cpp)

ament_target_dependencies(serial_reader ${dependencies})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS serial_reader DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch params DESTINATION share/${PROJECT_NAME})

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
