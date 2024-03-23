cmake_minimum_required(VERSION 3.8)
project(oriens_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(oriens_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

set(dependencies
  rclcpp
  sensor_msgs
  oriens_msgs
  Eigen3
)

include_directories(include)

add_library(oriens_core SHARED
  src/core.cpp
  src/sensor.cpp
  src/accelerometer.cpp
  src/gyroscope.cpp
  src/magnetometer.cpp
)

add_executable(oriens_core_exe
  src/core_node.cpp
)

ament_target_dependencies(oriens_core_exe ${dependencies})
target_link_libraries(oriens_core_exe oriens_core)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(TARGETS oriens_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/${PROJECT_NAME}
)

ament_target_dependencies(oriens_core ${dependencies})
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_export_libraries(oriens_core)

install(TARGETS oriens_core_exe DESTINATION lib/${PROJECT_NAME})


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
